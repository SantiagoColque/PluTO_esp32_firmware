# Host build of the decoder and its tests: make -C components/coord_dto/test_host
# The sanitizers catch reads past the buffer and unaligned accesses.
CC ?= cc
CFLAGS ?= -std=gnu17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -fno-sanitize-recover=all

test: test_coord_dto
	./test_coord_dto

test_coord_dto: test_coord_dto.c ../coord_dto.c ../include/coord_dto.h
	$(CC) $(CFLAGS) -I../include test_coord_dto.c ../coord_dto.c -o $@

clean:
	rm -f test_coord_dto

.PHONY: test clean
