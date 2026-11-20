#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证「设备私钥是否由 MAC / 设备信息 / PIN 等公开信息派生」。

原理：如果私钥是确定性派生的，那么用公开信息 + 已知派生算法算出的私钥，
      其对应公钥必然等于设备证书里那把。所以穷举所有合理组合，比对公钥即可
      —— 不需要知道真实算法，只需要验证结果。
      全部不中 → 私钥是产线随机生成，不可能从公开信息推导。

用法：
    # 从设备证书自动取公钥（推荐）
    python derive_test.py --cert device_cert.der --mac 11:22:33:44:55:66

    # 直接给公钥
    python derive_test.py --pubkey <X hex> --pubkey-y <Y hex> --mac 11:22:33:44:55:66

    # 设备外壳上有 PIN / SN 时一起加进去
    python derive_test.py --cert device_cert.der --mac ... --pin 123456 --sn ABC123

依赖：cryptography
"""
import argparse
import hashlib
import hmac
import itertools
import sys

from cryptography.hazmat.primitives.asymmetric import ec

CURVE = ec.SECP256R1()
# P-256 曲线阶
ORDER = 0xFFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551

CONSTS = [b"mijia", b"miio", b"xiaomi", b"miot", b"mible", b"mesh", b"mhc",
          b"Mijia", b"MIJIA", b"MiBeacon"]


def load_pubkey_from_cert(path):
    from cryptography import x509
    with open(path, "rb") as f:
        der = f.read()
    if b"-----BEGIN" in der:
        cert = x509.load_pem_x509_certificate(der)
    else:
        cert = x509.load_der_x509_certificate(der)
    n = cert.public_key().public_numbers()
    return n.x, n.y


def pub_of(k):
    k %= ORDER
    if k == 0:
        return None
    try:
        n = ec.derive_private_key(k, CURVE).public_key().public_numbers()
    except Exception:
        return None
    return (n.x, n.y)


def candidates(name, s):
    """产出一个种子的各种派生候选"""
    if len(s) >= 32:
        yield f"{name}[:32]", s[:32]
        yield f"{name}[-32:]", s[-32:]
    as_int = int.from_bytes(s, "big")
    if 0 < as_int < ORDER:
        yield f"{name} 整体当标量", s.rjust(32, b"\x00")
    yield f"sha256({name})", hashlib.sha256(s).digest()
    yield f"sha256({name}|{name})", hashlib.sha256(s + s).digest()
    yield f"sha512({name})[:32]", hashlib.sha512(s).digest()[:32]
    yield f"sha256(00|{name})", hashlib.sha256(b"\x00" + s).digest()
    yield f"sha256(01|{name})", hashlib.sha256(b"\x01" + s).digest()
    yield f"sha256({name}|00)", hashlib.sha256(s + b"\x00").digest()
    yield f"double-sha256({name})", hashlib.sha256(hashlib.sha256(s).digest()).digest()
    for c in CONSTS:
        yield f"sha256({name}|{c.decode()})", hashlib.sha256(s + c).digest()
        yield f"sha256({c.decode()}|{name})", hashlib.sha256(c + s).digest()
        yield f"hmac({c.decode()},{name})", hmac.new(c, s, hashlib.sha256).digest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cert", help="设备证书（DER 或 PEM），从中提取公钥")
    ap.add_argument("--pubkey", help="公钥 X（hex，未压缩点去掉 04 前缀也可）")
    ap.add_argument("--pubkey-y", help="公钥 Y（hex），给了 X 就必须给 Y")
    ap.add_argument("--mac", help="设备 MAC，如 11:22:33:44:55:66")
    ap.add_argument("--devinfo", help="设备信息段（hex），如 000000000000000000000102")
    ap.add_argument("--pin", help="设备 PIN / 配网码（外壳丝印上有的话）")
    ap.add_argument("--sn", help="设备 SN")
    a = ap.parse_args()

    if a.cert:
        tx, ty = load_pubkey_from_cert(a.cert)
    elif a.pubkey and a.pubkey_y:
        tx, ty = int(a.pubkey, 16), int(a.pubkey_y, 16)
    else:
        ap.error("需要 --cert 或同时给 --pubkey 与 --pubkey-y")

    seeds = []
    if a.mac:
        raw = bytes.fromhex(a.mac.replace(":", "").replace("-", ""))
        if len(raw) != 6:
            ap.error("MAC 需要 6 字节")
        seeds += [("MAC_BE", raw), ("MAC_LE", raw[::-1]),
                  ("MAC_ascii", a.mac.upper().replace("-", ":").encode())]
    if a.devinfo:
        seeds.append(("DEV_INFO", bytes.fromhex(a.devinfo)))
    if a.pin:
        seeds += [("PIN_ascii", a.pin.encode()), ("PIN_hex", bytes.fromhex(a.pin))]
    if a.sn:
        seeds.append(("SN_ascii", a.sn.encode()))
    seeds.append(("PUB_XY", tx.to_bytes(32, "big") + ty.to_bytes(32, "big")))

    if not seeds:
        ap.error("至少给一个种子：--mac / --devinfo / --pin / --sn")

    base = list(seeds)
    for (n1, s1), (n2, s2) in itertools.combinations(base, 2):
        if len(s1) + len(s2) <= 96:
            seeds.append((f"{n1}+{n2}", s1 + s2))

    tried, seen, hits = 0, set(), []
    for name, s in seeds:
        if not s:
            continue
        for desc, k in candidates(name, s):
            if k in seen:
                continue
            seen.add(k)
            tried += 1
            if pub_of(int.from_bytes(k, "big")) == (tx, ty):
                hits.append(desc)
                print(f"\n*** 命中 *** {desc}\n  私钥 {k.hex()}")

    print(f"\n种子 {len(seeds)} 个，候选 {tried} 个，命中 {len(hits)} 个")
    if not hits:
        print("结论：私钥不是由公开信息派生 —— 产线随机生成，无法推导")
    return 0 if hits else 1


if __name__ == "__main__":
    sys.exit(main())
