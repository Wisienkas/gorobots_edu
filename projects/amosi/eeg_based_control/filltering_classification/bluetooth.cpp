#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <time.h>
#include <numeric>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>

#include "bluetooth.h"

using namespace std;

void Bluetooth::init() {
	fd = open("/dev/rfcomm0", O_RDWR | O_NOCTTY);
}

void Bluetooth::shutdown() {
	close(fd);
}

int Bluetooth::readValue() {
	char line[1000];
	int n = 0, spot = 0;
	char buf = '\0';
	memset(line, '\0', sizeof line);

	do {
		n = read(fd, &buf, 1);
		sprintf(&line[spot], "%c", buf);
		spot += n;
	} while ((buf != '\r' && buf != '\n') && n > 0);

	sprintf(&line[spot-1], "%c", '\0');

	if (strlen(line) <= 0 || line[0] == '\n')
		return 0;

	return stoi(line);

}
