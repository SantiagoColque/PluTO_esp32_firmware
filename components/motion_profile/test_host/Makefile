# Host build of the motion profile and its tests: make -C components/motion_profile/test_host
CC ?= cc
CFLAGS ?= -std=gnu17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -fno-sanitize-recover=all

test: test_motion_profile
	./test_motion_profile

test_motion_profile: test_motion_profile.c ../motion_profile.c ../include/motion_profile.h
	$(CC) $(CFLAGS) -I../include test_motion_profile.c ../motion_profile.c -lm -o $@

clean:
	rm -f test_motion_profile

.PHONY: test clean
