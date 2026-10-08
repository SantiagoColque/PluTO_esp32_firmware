# Host build of the count-to-angle conversion and its tests: make -C components/encoder/test_host
# Only the pure half is built; as5600.c needs the I2C driver.
CC ?= cc
CFLAGS ?= -std=gnu17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -fno-sanitize-recover=all

test: test_encoder_angle
	./test_encoder_angle

test_encoder_angle: test_encoder_angle.c ../encoder_angle.c ../include/encoder_angle.h
	$(CC) $(CFLAGS) -I../include test_encoder_angle.c ../encoder_angle.c -o $@

clean:
	rm -f test_encoder_angle

.PHONY: test clean
