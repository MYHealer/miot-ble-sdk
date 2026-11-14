#!/usr/bin/env bash
# 全量构建自检（回归门禁）—— 编译 esp32/ 下所有含 CMakeLists.txt 的工程
#
# 为什么需要：SDK 的对外函数签名一改（例如 scale_s400_store_offline 加 ts 参数），
# 没被编译到的工程会静默变成「编译不过」，等下次单独烧它时才发现。2026-11-14
# 就是这样：体脂秤改造后 multi_dev 两处旧调用失效，藏了几个小时。
#
# 用法（在仓库根目录）：
#   bash tools/build_all.sh              # 全量编译，结果打到 /tmp，汇总在末尾
#   bash tools/build_all.sh scale_adv    # 只编指定工程
#
# multi_dev 自动走 esp32/multi_dev/build_mesh.sh（需要 6 个 NimBLE 移植补丁），
# 其余工程走 idf.py build。全部在 espressif/idf:v6.0 容器内完成，不动本机环境。
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd -W 2>/dev/null)"
[ -n "$REPO" ] || REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMG="${IDF_IMAGE:-espressif/idf:v6.0}"
LOG="${BUILD_ALL_LOG:-/tmp/build_all.log}"
ONLY="${1:-}"
: > "$LOG"

PASS=""
FAIL=""
n_ok=0
n_fail=0

for d in "$REPO"/esp32/*/; do
    n=$(basename "$d")
    [ -f "$d/CMakeLists.txt" ] || continue
    if [ -n "$ONLY" ] && [ "$n" != "$ONLY" ]; then continue; fi
    t0=$(date +%s)
    echo "##### $n #####" >> "$LOG"

    if [ -f "$d/build_mesh.sh" ]; then
        # Mesh 工程：自带补丁与容器调用
        ( cd "$d" && bash build_mesh.sh ) >> "$LOG" 2>&1
        rc=$?
        blog="$d/build/mesh_build.log"
    else
        MSYS_NO_PATHCONV=1 docker run --rm -v "$REPO:/project" -w "/project/esp32/$n" "$IMG" \
            bash -c 'idf.py build > build/all.log 2>&1; rc=$?; grep -E "error|Project build complete" build/all.log | tail -5; exit $rc' >> "$LOG" 2>&1
        rc=$?
        blog="$d/build/all.log"
    fi

    t1=$(date +%s)
    bin=$(ls -l "$d"/build/*.bin 2>/dev/null | awk '{print $5}' | sort -n | tail -1)
    warn=$(grep -c "warning:" "$blog" 2>/dev/null || true)
    warn=${warn:-0}

    if [ $rc -eq 0 ]; then
        PASS="$PASS $n"; n_ok=$((n_ok + 1)); st=OK
    else
        FAIL="$FAIL $n"; n_fail=$((n_fail + 1)); st=FAIL
    fi
    printf '%-16s %-4s %4ss  %8s B  warn=%s\n' "$n" "$st" "$((t1 - t0))" "${bin:-0}" "$warn" | tee -a "$LOG"
done

echo "" | tee -a "$LOG"
[ $((n_ok + n_fail)) -gt 0 ] || { echo "没匹配到工程（检查名字或路径）" | tee -a "$LOG"; exit 2; }
echo "PASS ($n_ok):$PASS" | tee -a "$LOG"
[ $n_fail -eq 0 ] || echo "FAIL ($n_fail):$FAIL" | tee -a "$LOG"
echo "完整日志：$LOG" | tee -a "$LOG"
[ $n_fail -eq 0 ]
