# Host build of the trajectory logic and its tests: make -C components/trajectory/test_host
CC ?= cc
CFLAGS ?= -std=gnu17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -fno-sanitize-recover=all

test: test_trajectory
	./test_trajectory

test_trajectory: test_trajectory.c ../trajectory.c ../include/trajectory.h
	$(CC) $(CFLAGS) -I../include test_trajectory.c ../trajectory.c -o $@

clean:
	rm -f test_trajectory

.PHONY: test clean
