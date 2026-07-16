CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -std=c11

eps: main.c
	$(CC) $(CFLAGS) -o $@ $< -lm

clean:
	rm -f eps

.PHONY: clean
