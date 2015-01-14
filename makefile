prefix=/usr/local

all: PiCW

PiCW: PiCW.cpp
	g++ -std=c++11 -Wall -Werror -lm PiCW.cpp -oPiCW

#.PHONY: install
#install: wspr
#	install -m 0755 wspr $(prefix)/bin
#	install -m 0755 gpioclk $(prefix)/bin

#.PHONY: uninstall
#uninstall:
#	rm -f $(prefix)/bin/wspr
#	rm -f $(prefix)/bin/gpioclk

