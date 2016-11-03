#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <signal.h>

#define SOP 0x11
#define EOP 0x14
#define DLE 0x10

#define CMD_DRIVE 0x04

using namespace std;

static volatile int keepRunning = 1;
int serial_port;

/* Internal Function Prototypes */
int open_port(void);
void configure_port(void);
void close_port(void);
void intHandler(int dummy);

void intHandler(int dummy) {
	keepRunning = 0;
}

/* Funciton Implementation */
int open_port(void) {
	int fd;
	fd = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

	if (fd == -1) {
		printf("Failed to open port!!!\n");
	} else {
		fcntl(fd, F_SETFL, 0);
	}

	return fd;
}

void configure_port(void) {
	struct termios port_settings;
	memset (&port_settings, 0, sizeof port_settings);

	cfsetispeed(&port_settings, B9600);
	cfsetospeed(&port_settings, B9600);

	port_settings.c_cflag &= ~PARENB;
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;

	port_settings.c_cflag &= ~CRTSCTS;
	port_settings.c_cc[VMIN] = 1;
	port_settings.c_cc[VTIME] = 5;
	port_settings.c_cflag |= CREAD | CLOCAL;

	cfmakeraw(&port_settings);

	tcflush(serial_port, TCIFLUSH);

	if (tcsetattr(serial_port, TCSANOW, &port_settings) != 0)printf("ERROR\n");
}

void close_port(void) {
	close(serial_port);
}

void send_drive_message(uint16_t left, uint16_t right) {
	uint8_t payload[255];
	uint8_t index;
	uint8_t lh = (left >> 8) & 0xFF;
	uint8_t ll = (left) & 0xFF;
	uint8_t rh = (right >> 8) & 0xFF;
	uint8_t rl = (right) & 0xFF;

	payload[0] = SOP;
	payload[1] = CMD_DRIVE;
	index = 2;

	if ((lh == SOP) || (lh == EOP) || (lh == DLE)) {
		payload[index] = DLE;
		payload[index+1] = lh;
		index += 2;
	} else {
		payload[index] = lh;
		index += 1;
	}

	if ((ll == SOP) || (ll == EOP) || (ll == DLE)) {
		payload[index] = DLE;
		payload[index+1] = ll;
		index += 2;
	} else {
		payload[index] = ll;
		index += 1;
	}

	if ((rh == SOP) || (rh == EOP) || (rh == DLE)) {
		payload[index] = DLE;
		payload[index+1] = rh;
		index += 2;
	} else {
		payload[index] = rh;
		index += 1;
	}

	if ((rl == SOP) || (rl == EOP) || (rl == DLE)) {
		payload[index] = DLE;
		payload[index+1] = rl;
		index += 2;
	} else {
		payload[index] = rl;
		index += 1;
	}

	payload[index] = EOP;
	index++;

	write(serial_port, payload, index);
}

int main() {
	serial_port = open_port();
	printf("Serial id - %d\n", serial_port);

	if(serial_port > 0) {
		configure_port();
	} else {
		return -1;
	}

	unsigned long long currentTime = 0;
	double velocity = 0;
	double turn = 0;
	int hertz = 0;
	int16_t counter = 0;

	signal(SIGINT, intHandler);

	while (keepRunning) {
		//Bound to legal data
		if (velocity > 1.0) velocity = 1.0;
		if (velocity < -1.0) velocity = -1.0;
		if (turn > 1.0) turn = 1.0;
		if (turn < -1.0) turn = -1.0;

		int16_t left = (1024 + (velocity * 1023)) + (turn * 1023);
		int16_t right = (1024 + (velocity * 1023)) - (turn * 1023);

		if (left < 0) left = 0;
		if (left > 2047) left = 2047;
		if (right < 0) right = 0;
		if (right > 2047) right = 2047;

		fprintf(stdout, "LEFT %d; RIGHT %d\n", left, right);
		send_drive_message((uint16_t)left, (uint16_t)right);
		usleep(100000);
		velocity+= 0.01f;
	}
	return 0;
}
