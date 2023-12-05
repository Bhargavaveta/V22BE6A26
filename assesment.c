#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

int main() {
    int serialPort;
    struct termios tty;
    char buffer[100];

    // Replace "/dev/ttyUSB0" with the actual path of your serial port
    serialPort = open("/dev/ttyUSB0", O_RDWR);

    if (serialPort == -1) {
        perror("Error opening serial port");
        return 1;
    }

    if (tcgetattr(serialPort, &tty) != 0) {
        perror("Error getting serial port attributes");
        close(serialPort);
        return 1;
    }

    cfsetospeed(&tty, B9600);  // Set your baud rate
    cfsetispeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and ignore modem control lines
    tty.c_cflag &= ~PARENB;           // No parity bit
    tty.c_cflag &= ~CSTOPB;           // 1 stop bit
    tty.c_cflag &= ~CSIZE;            // Clear data size bits
    tty.c_cflag |= CS8;               // 8 bits per byte

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        perror("Error setting serial port attributes");
        close(serialPort);
        return 1;
    }

    while (1) {
        ssize_t bytesRead = read(serialPort, buffer, sizeof(buffer));

        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';  // Null-terminate the received data
            printf("Received data: %s\n", buffer);
        }
    }

    close(serialPort);

    return 0;
}
