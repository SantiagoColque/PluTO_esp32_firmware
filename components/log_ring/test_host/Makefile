# Host build of the ring and its tests: make -C components/log_ring/test_host
# Only the pure half is built; log_ring.c needs FreeRTOS and esp_log.
CC ?= cc
CFLAGS ?= -std=gnu17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -fno-sanitize-recover=all

test: test_log_ring
	./test_log_ring

test_log_ring: test_log_ring.c ../log_ring_core.c ../include/log_ring_core.h
	$(CC) $(CFLAGS) -I../include test_log_ring.c ../log_ring_core.c -o $@

clean:
	rm -f test_log_ring

.PHONY: test clean
