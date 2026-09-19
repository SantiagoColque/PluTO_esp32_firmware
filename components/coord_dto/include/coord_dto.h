#ifndef COORD_DTO_H
#define COORD_DTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Decoder for the pointing payload on device/<device_id>/coordinates/polar.
 * The format is defined in PluTO's docs/contracts/coordinates-dto.md, and the
 * section numbers below refer to it. It has no ESP-IDF dependencies on purpose,
 * so the same code also runs as a host test (test_host/).
 */

#define PLUTO_DTO_MAGIC       0x50
#define PLUTO_DTO_FLAG_HOLD   0x01
#define PLUTO_DTO_MAX_POINTS  16
#define PLUTO_DTO_HEADER_LEN  19
#define PLUTO_DTO_POINT_LEN   8
#define PLUTO_DTO_AZ_MAX_CDEG 35999
#define PLUTO_DTO_EL_MAX_CDEG 9000

/* Wire layout (§3). Only ever filled through memcpy, never by casting a
 * pointer into the received buffer (§6.2). */
typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t flags;
    uint8_t count;
    int64_t t0_ms;
    int64_t t_sent_ms;
} pluto_dto_header_t;

typedef struct __attribute__((packed)) {
    uint32_t dt_ms;
    uint16_t az_cdeg;
    int16_t el_cdeg;
} pluto_dto_point_t;

_Static_assert(sizeof(pluto_dto_header_t) == PLUTO_DTO_HEADER_LEN, "header must be 19 bytes");
_Static_assert(sizeof(pluto_dto_point_t) == PLUTO_DTO_POINT_LEN, "point must be 8 bytes");

typedef enum {
    COORD_DTO_OK = 0,
    COORD_DTO_ERR_LENGTH,
    COORD_DTO_ERR_MAGIC,
    COORD_DTO_ERR_FLAGS,
    COORD_DTO_ERR_COUNT,
    COORD_DTO_ERR_RANGE,
    COORD_DTO_ERR_ORDER,
} coord_dto_err_t;

typedef struct {
    uint32_t dt_ms;
    uint16_t az_cdeg;
    int16_t el_cdeg;
} coord_dto_point_t;

typedef struct {
    uint8_t flags;
    uint8_t count;
    int64_t t0_ms;
    int64_t t_sent_ms;
    coord_dto_point_t points[PLUTO_DTO_MAX_POINTS];
} coord_dto_msg_t;

/**
 * @brief Decode and validate one payload (§6.1 and §6.5).
 *
 * Checks the length before reading any point, so a malformed payload can never
 * be read past its end. `out` is only meaningful when the result is
 * COORD_DTO_OK. The time-based rules (expired points, unsynced clock) need the
 * current time and belong to the caller.
 */
coord_dto_err_t coord_dto_decode(const uint8_t *payload, size_t payload_len, coord_dto_msg_t *out);

bool coord_dto_is_hold(const coord_dto_msg_t *msg);

/**
 * @brief Reject code reported on the state channel (device-state.md §5).
 *
 * Returns NULL for COORD_DTO_OK.
 */
const char *coord_dto_err_code(coord_dto_err_t err);

#endif
