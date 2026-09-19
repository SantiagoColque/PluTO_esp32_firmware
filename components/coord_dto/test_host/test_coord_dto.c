#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "coord_dto.h"

static int s_failures;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            s_failures++;                                                        \
        }                                                                        \
    } while (0)

/* The worked example of docs/contracts/coordinates-dto.md §7. */
static const char *EXAMPLE_HEX =
    "500003403136b6a0010000702936b6a0010000"
    "0000000039309411"
    "e80300007030b211"
    "d0070000a730d011";
static const char *HOLD_HEX = "500100403136b6a0010000702936b6a0010000";

static size_t from_hex(const char *hex, uint8_t *out)
{
    size_t len = strlen(hex) / 2;

    for (size_t i = 0; i < len; i++) {
        unsigned int byte;
        sscanf(hex + 2 * i, "%2x", &byte);
        out[i] = (uint8_t)byte;
    }
    return len;
}

/* A valid batch of `count` points, one second apart, to corrupt field by field. */
static size_t build_batch(uint8_t *buf, uint8_t count)
{
    pluto_dto_header_t header = {PLUTO_DTO_MAGIC, 0, count, 1789763400000LL, 1789763398000LL};

    memcpy(buf, &header, sizeof(header));
    for (uint8_t i = 0; i < count; i++) {
        pluto_dto_point_t point = {(uint32_t)i * 1000u, (uint16_t)(12345 + i), 4500};
        memcpy(buf + PLUTO_DTO_HEADER_LEN + (size_t)i * PLUTO_DTO_POINT_LEN, &point, sizeof(point));
    }
    return PLUTO_DTO_HEADER_LEN + (size_t)count * PLUTO_DTO_POINT_LEN;
}

static void set_point(uint8_t *buf, uint8_t index, uint32_t dt_ms, uint16_t az, int16_t el)
{
    pluto_dto_point_t point = {dt_ms, az, el};
    memcpy(buf + PLUTO_DTO_HEADER_LEN + (size_t)index * PLUTO_DTO_POINT_LEN, &point, sizeof(point));
}

static void test_decodes_the_worked_example(void)
{
    uint8_t buf[64];
    coord_dto_msg_t msg;
    size_t len = from_hex(EXAMPLE_HEX, buf);

    CHECK(len == 43);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_OK);
    CHECK(!coord_dto_is_hold(&msg));
    CHECK(msg.count == 3);
    CHECK(msg.t0_ms == 1789763400000LL);
    CHECK(msg.t_sent_ms == 1789763398000LL);
    CHECK(msg.points[0].dt_ms == 0 && msg.points[0].az_cdeg == 12345 && msg.points[0].el_cdeg == 4500);
    CHECK(msg.points[1].dt_ms == 1000 && msg.points[1].az_cdeg == 12400 && msg.points[1].el_cdeg == 4530);
    CHECK(msg.points[2].dt_ms == 2000 && msg.points[2].az_cdeg == 12455 && msg.points[2].el_cdeg == 4560);
}

static void test_decodes_a_hold(void)
{
    uint8_t buf[32];
    coord_dto_msg_t msg;
    size_t len = from_hex(HOLD_HEX, buf);

    CHECK(len == 19);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_OK);
    CHECK(coord_dto_is_hold(&msg));
    CHECK(msg.count == 0);
}

static void test_decodes_from_an_unaligned_buffer(void)
{
    uint8_t *raw = malloc(64 + 1);
    coord_dto_msg_t msg;
    size_t len = from_hex(EXAMPLE_HEX, raw + 1);

    CHECK(coord_dto_decode(raw + 1, len, &msg) == COORD_DTO_OK);
    CHECK(msg.points[2].az_cdeg == 12455);
    free(raw);
}

static void test_rejects_bad_length(void)
{
    uint8_t buf[256];
    coord_dto_msg_t msg;
    size_t len = build_batch(buf, 3);

    CHECK(coord_dto_decode(buf, 18, &msg) == COORD_DTO_ERR_LENGTH);
    CHECK(coord_dto_decode(NULL, 0, &msg) == COORD_DTO_ERR_LENGTH);
    CHECK(coord_dto_decode(buf, len - 1, &msg) == COORD_DTO_ERR_LENGTH);
    CHECK(coord_dto_decode(buf, len + 1, &msg) == COORD_DTO_ERR_LENGTH);
}

static void test_rejects_bad_header(void)
{
    uint8_t buf[256];
    coord_dto_msg_t msg;
    size_t len = build_batch(buf, 3);

    buf[0] = 0x51;
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_MAGIC);
    buf[0] = PLUTO_DTO_MAGIC;

    buf[1] = 0x02;
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_FLAGS);
    buf[1] = 0x80;
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_FLAGS);
}

static void test_rejects_bad_count(void)
{
    uint8_t buf[256];
    coord_dto_msg_t msg;
    size_t len;

    len = build_batch(buf, 17);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_COUNT);

    len = build_batch(buf, 0);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_COUNT);

    len = build_batch(buf, 1);
    buf[1] = PLUTO_DTO_FLAG_HOLD;
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_COUNT);
}

static void test_accepts_the_largest_batch(void)
{
    uint8_t buf[256];
    coord_dto_msg_t msg;
    size_t len = build_batch(buf, PLUTO_DTO_MAX_POINTS);

    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_OK);
    CHECK(msg.count == PLUTO_DTO_MAX_POINTS);
}

static void test_rejects_out_of_range_angles(void)
{
    uint8_t buf[256];
    coord_dto_msg_t msg;
    size_t len = build_batch(buf, 3);

    set_point(buf, 1, 1000, 36000, 4500);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_RANGE);
    set_point(buf, 1, 1000, 35999, 9001);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_RANGE);
    set_point(buf, 1, 1000, 35999, -9001);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_RANGE);

    set_point(buf, 1, 1000, 35999, -9000);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_OK);
}

static void test_rejects_points_out_of_order(void)
{
    uint8_t buf[256];
    coord_dto_msg_t msg;
    size_t len = build_batch(buf, 3);

    set_point(buf, 2, 1000, 12455, 4560);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_ORDER);
    set_point(buf, 2, 500, 12455, 4560);
    CHECK(coord_dto_decode(buf, len, &msg) == COORD_DTO_ERR_ORDER);
}

static void test_error_codes_match_the_state_channel(void)
{
    CHECK(coord_dto_err_code(COORD_DTO_OK) == NULL);
    CHECK(strcmp(coord_dto_err_code(COORD_DTO_ERR_LENGTH), "bad_length") == 0);
    CHECK(strcmp(coord_dto_err_code(COORD_DTO_ERR_MAGIC), "bad_magic") == 0);
    CHECK(strcmp(coord_dto_err_code(COORD_DTO_ERR_FLAGS), "reserved_flags") == 0);
    CHECK(strcmp(coord_dto_err_code(COORD_DTO_ERR_COUNT), "bad_count") == 0);
    CHECK(strcmp(coord_dto_err_code(COORD_DTO_ERR_RANGE), "out_of_range") == 0);
    CHECK(strcmp(coord_dto_err_code(COORD_DTO_ERR_ORDER), "not_increasing") == 0);
}

int main(void)
{
    test_decodes_the_worked_example();
    test_decodes_a_hold();
    test_decodes_from_an_unaligned_buffer();
    test_rejects_bad_length();
    test_rejects_bad_header();
    test_rejects_bad_count();
    test_accepts_the_largest_batch();
    test_rejects_out_of_range_angles();
    test_rejects_points_out_of_order();
    test_error_codes_match_the_state_channel();

    if (s_failures) {
        fprintf(stderr, "coord_dto: %d check(s) failed\n", s_failures);
        return 1;
    }
    printf("coord_dto: all tests passed\n");
    return 0;
}
