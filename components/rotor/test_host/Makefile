# Host build of the rotor logic and its tests: make -C components/rotor/test_host
# It links the pure components it depends on.
CC ?= cc
CFLAGS ?= -std=gnu17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -fno-sanitize-recover=all
COMPONENTS = ../..
INCLUDES = -I../include -I$(COMPONENTS)/coord_dto/include -I$(COMPONENTS)/trajectory/include -I$(COMPONENTS)/pan_tilt/include
SOURCES = ../rotor.c $(COMPONENTS)/coord_dto/coord_dto.c $(COMPONENTS)/trajectory/trajectory.c $(COMPONENTS)/pan_tilt/pan_tilt.c

test: test_rotor
	./test_rotor

test_rotor: test_rotor.c $(SOURCES) ../include/rotor.h
	$(CC) $(CFLAGS) $(INCLUDES) test_rotor.c $(SOURCES) -o $@

clean:
	rm -f test_rotor

.PHONY: test clean
