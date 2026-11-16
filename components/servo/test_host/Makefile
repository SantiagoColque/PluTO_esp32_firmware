# Host build of the angle-to-pulse conversion and its tests: make -C components/servo/test_host
# Only the pure half is built; servo_driver.c needs LEDC.
CC ?= cc
CFLAGS ?= -std=gnu17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -fno-sanitize-recover=all

test: test_servo_pulse
	./test_servo_pulse

test_servo_pulse: test_servo_pulse.c ../servo_pulse.c ../include/servo_pulse.h
	$(CC) $(CFLAGS) -I../include test_servo_pulse.c ../servo_pulse.c -o $@

clean:
	rm -f test_servo_pulse

.PHONY: test clean
