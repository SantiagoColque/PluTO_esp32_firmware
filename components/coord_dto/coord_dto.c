#include "coord_dto.h"

#include <string.h>

coord_dto_err_t coord_dto_decode(const uint8_t *payload, size_t payload_len, coord_dto_msg_t *out)
{
    pluto_dto_header_t header;
    bool hold;

    if (payload == NULL || payload_len < PLUTO_DTO_HEADER_LEN) {
        return COORD_DTO_ERR_LENGTH;
    }

    /* esp-mqtt's buffer has no alignment guarantee, and an unaligned 32- or
     * 64-bit access faults on Xtensa, so every field is copied out (§6.2). */
    memcpy(&header, payload, sizeof(header));

    if (header.magic != PLUTO_DTO_MAGIC) {
        return COORD_DTO_ERR_MAGIC;
    }
    if (header.flags & ~PLUTO_DTO_FLAG_HOLD) {
        return COORD_DTO_ERR_FLAGS;
    }

    hold = (header.flags & PLUTO_DTO_FLAG_HOLD) != 0;
    if (header.count > PLUTO_DTO_MAX_POINTS) {
        return COORD_DTO_ERR_COUNT;
    }
    if (header.count == 0 && !hold) {
        return COORD_DTO_ERR_COUNT;
    }
    if (header.count > 0 && hold) {
        return COORD_DTO_ERR_COUNT;
    }
    if (payload_len != PLUTO_DTO_HEADER_LEN + (size_t)header.count * PLUTO_DTO_POINT_LEN) {
        return COORD_DTO_ERR_LENGTH;
    }

    for (uint8_t i = 0; i < header.count; i++) {
        pluto_dto_point_t wire;

        memcpy(&wire, payload + PLUTO_DTO_HEADER_LEN + (size_t)i * PLUTO_DTO_POINT_LEN, sizeof(wire));

        if (wire.az_cdeg > PLUTO_DTO_AZ_MAX_CDEG ||
            wire.el_cdeg < -PLUTO_DTO_EL_MAX_CDEG || wire.el_cdeg > PLUTO_DTO_EL_MAX_CDEG) {
            return COORD_DTO_ERR_RANGE;
        }
        if (i > 0 && wire.dt_ms <= out->points[i - 1].dt_ms) {
            return COORD_DTO_ERR_ORDER;
        }

        out->points[i].dt_ms = wire.dt_ms;
        out->points[i].az_cdeg = wire.az_cdeg;
        out->points[i].el_cdeg = wire.el_cdeg;
    }

    out->flags = header.flags;
    out->count = header.count;
    out->t0_ms = header.t0_ms;
    out->t_sent_ms = header.t_sent_ms;
    return COORD_DTO_OK;
}

bool coord_dto_is_hold(const coord_dto_msg_t *msg)
{
    return (msg->flags & PLUTO_DTO_FLAG_HOLD) != 0;
}

const char *coord_dto_err_code(coord_dto_err_t err)
{
    switch (err) {
    case COORD_DTO_OK:
        return NULL;
    case COORD_DTO_ERR_LENGTH:
        return "bad_length";
    case COORD_DTO_ERR_MAGIC:
        return "bad_magic";
    case COORD_DTO_ERR_FLAGS:
        return "reserved_flags";
    case COORD_DTO_ERR_COUNT:
        return "bad_count";
    case COORD_DTO_ERR_RANGE:
        return "out_of_range";
    case COORD_DTO_ERR_ORDER:
        return "not_increasing";
    }
    return "unknown";
}
