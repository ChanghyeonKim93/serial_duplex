// MAIN_TX.CPP

#include <iostream>
#include <stdio.h>
#include <stdlib.h> // atoi itoa
#include <cstdlib>

#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>

#include <sys/time.h>

#define PI 3.141592

int main(void) {
	int fd_parani1;
	struct termios newtio_tx;
	printf("This is PARANI1 (TX) node.\n");
	fd_parani1 = open("/dev/PARANI1", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd_parani1 == -1) {
		printf("Error - no serial port /dev/PARANI1 is opened! Check the serial device.\n");
		return -1;
	}
	else
		printf("/dev/PARANI1 is opened.\n");
	memset(&newtio_tx, 0, sizeof(newtio_tx));
	newtio_tx.c_cflag = B115200;
	newtio_tx.c_cflag |= CS8;
	newtio_tx.c_cflag |= CLOCAL | CREAD;
	newtio_tx.c_iflag = 0; // no parity
	newtio_tx.c_oflag = 0;
	newtio_tx.c_lflag = 0;
	newtio_tx.c_cc[VTIME] = 0;
	newtio_tx.c_cc[VMIN] = 1;

	tcflush(fd_parani1, TCIFLUSH);
	tcsetattr(fd_parani1, TCSANOW, &newtio_tx);

	// write
	int cnt = 1;
	char buf_write[256];
	int T = 20; // [ms]
	clock_t start, finish;
	write(fd_parani1, "%%%%%%%dump%%%%%%", 19);
	while (1) {
		start = clock();
		int n;
		n=sprintf(buf_write,"(#%07d&data1&data2&data3&data4&data5#)",cnt);
		write(fd_parani1,buf_write,n);
		printf("write tx cnt : %d\n", cnt);
		cnt++;
		usleep(1000 * T);
		finish = clock();
		double duration = (double)(finish - start) / 1000.0;
		printf("TX : %f [ms]\n", duration + T);
	}

	close(fd_parani1);
	return 0;
}