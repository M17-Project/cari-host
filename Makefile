all: cari-host

cari-host: cari-host.c
	gcc -O2 -Wall -Wextra cari-host.c -o cari-host -lm -lzmq

install: all
	sudo install cari-host /usr/local/bin

clean:
	rm -f cari-host