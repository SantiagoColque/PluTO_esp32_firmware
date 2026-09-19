# Host build of the pan-tilt mapping and its tests: make -C components/pan_tilt/test_host
CC ?= cc
CFLAGS ?= -std=gnu17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -fno-sanitize-recover=all

test: test_pan_tilt
	./test_pan_tilt

test_pan_tilt: test_pan_tilt.c ../pan_tilt.c ../include/pan_tilt.h
	$(CC) $(CFLAGS) -I../include test_pan_tilt.c ../pan_tilt.c -o $@

clean:
	rm -f test_pan_tilt

.PHONY: test clean
