/**
 * @file miot_props.c
 * @brief 属性表驱动执行器实现
 */
#include "miot_ble/miot_props.h"
#include <string.h>

miot_prop_t *miot_props_find(miot_prop_t *tbl, size_t cnt, uint8_t siid, uint16_t piid)
{
    for (size_t i = 0; i < cnt; i++) {
        if (tbl[i].siid == siid && tbl[i].piid == piid) return &tbl[i];
    }
    return NULL;
}

size_t miot_props_encode(const miot_prop_t *p, uint8_t *out, size_t cap)
{
    if (cap < 6) return 0;
    uint8_t v[4], vlen;
    /* ★按 size 精确读：size=2(u16) 若按 int32 读会越界读相邻变量
     * （磁吸灯实测：读到的高位字节是邻居，编进包后值错乱）。 */
    int32_t iv;
    if (p->size == 1)      iv = (int32_t)(*(uint8_t *)p->ptr);
    else if (p->size == 2) iv = (int32_t)(*(uint16_t *)p->ptr);
    else                   iv = *(int32_t *)p->ptr;

    switch (p->type) {
    case MIOT_TYPE_BOOL:  v[0] = iv ? 1 : 0; vlen = 1; break;
    case MIOT_TYPE_U8:    v[0] = (uint8_t)iv; vlen = 1; break;
    case MIOT_TYPE_U16:   v[0] = (uint8_t)iv; v[1] = (uint8_t)(iv >> 8); vlen = 2; break;
    case MIOT_TYPE_U32:
    case MIOT_TYPE_I32:   memcpy(v, &iv, 4); vlen = 4; break;
    case MIOT_TYPE_FLOAT: { float f = (float)iv; memcpy(v, &f, 4); vlen = 4; } break;
    default: return 0;
    }
    uint16_t tl = (uint16_t)((p->type << 12) | vlen);
    out[0] = (uint8_t)(tl & 0xFF);
    out[1] = (uint8_t)(tl >> 8);
    memcpy(out + 2, v, vlen);
    return 2 + (size_t)vlen;
}

bool miot_props_apply(miot_prop_t *p, uint8_t vtype, const uint8_t *v, uint8_t vlen)
{
    if (p->rw != MIOT_PROP_RW) return false;

    int32_t iv;
    if (vtype == MIOT_TYPE_FLOAT && vlen >= 4) {
        float f; memcpy(&f, v, 4); iv = (int32_t)f;
    } else if (vlen == 1) {
        iv = v[0];
    } else if (vlen == 2) {
        iv = (int16_t)(uint16_t)(v[0] | (v[1] << 8));
    } else if (vlen >= 4) {
        iv = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) |
                       ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
    } else {
        return false;
    }
    if (p->max > p->min) {
        if (iv < p->min) iv = p->min;
        if (iv > p->max) iv = p->max;
    }
    if (p->type == MIOT_TYPE_BOOL) iv = iv ? 1 : 0;

    /* ★按 size 精确写：size=2(u16) 若写 4 字节 int32 会踩掉相邻变量。
     * 磁吸灯实测根因：拖色温 SET 7.3 → *(int32_t*)&v_ct 写 4 字节，
     * 把后面的 v_on/v_bright/v_light_mode 全踩成 0 → App GET 读到
     * bright=0，亮度突变归 0。 */
    if (p->size == 1)      *(uint8_t *)p->ptr  = (uint8_t)iv;
    else if (p->size == 2) *(uint16_t *)p->ptr = (uint16_t)iv;
    else                   *(int32_t *)p->ptr  = iv;

    if (p->on_set) p->on_set(p);
    return true;
}
