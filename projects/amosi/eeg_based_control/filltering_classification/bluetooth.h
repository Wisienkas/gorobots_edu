#ifndef BLUETOOTH_H
#define BLUETOOTH_H

using namespace std;

class Bluetooth {
private:
    int fd;
    const int baud = 9600;

public:
	void init();
	void shutdown();
	int readValue();
};

#endif
