prefix=/usr/local

all: PiCW

PiCW: PiCW.cpp
	g++-4.7 -D_GLIBCXX_DEBUG -std=c++11 -Wall -Werror -fmax-errors=5 -lm PiCW.cpp -pthread -oPiCW

.PHONY: install
install: PiCW
	install -m 0755 PiCW $(prefix)/bin

.PHONY: uninstall
uninstall:
	rm -f $(prefix)/bin/PiCW

