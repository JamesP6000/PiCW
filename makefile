prefix=/usr/local

all: PiCW

PiCW: PiCW.cpp
	g++-4.7 -D_GLIBCXX_DEBUG -std=c++11 -Wall -Werror -fmax-errors=5 -lm PiCW.cpp -oPiCW -pthread

#.PHONY: install
#install: wspr
#	install -m 0755 wspr $(prefix)/bin
#	install -m 0755 gpioclk $(prefix)/bin

#.PHONY: uninstall
#uninstall:
#	rm -f $(prefix)/bin/wspr
#	rm -f $(prefix)/bin/gpioclk

