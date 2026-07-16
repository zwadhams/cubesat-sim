CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -std=c11

obc: main.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f obc

.PHONY: clean
