prefix=/usr/local

archis = $(if $(findstring $(1),$(shell uname -m)),$(2))
pi_version_flag = $(if $(call archis,armv7,dummy-text),-DRPI2,-DRPI1)

all: PiCW

mailbox.o: mailbox.c mailbox.h
	g++ -c -Wall -lm mailbox.c

PiCW: PiCW.cpp mailbox.o
	g++ -D_GLIBCXX_DEBUG -std=c++11 -Wall -Werror -fmax-errors=5 -lm mailbox.o PiCW.cpp -pthread -oPiCW

clean:
	-rm PiCW
	-rm mailbox.o

.PHONY: install
install: PiCW
	install -m 0755 PiCW $(prefix)/bin

.PHONY: uninstall
uninstall:
	-rm -f $(prefix)/bin/PiCW

