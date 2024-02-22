#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h> // write(), read(), close()
#include <termios.h> // Contains POSIX terminal control definitions
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> //Error integer and strerror() function

#include "servo/servo_command.hpp"

// Function to configure serial port
int configure_serial_port(int serial_port) {
    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }
    return 0;
}

// Function to read from serial port
int read_from_serial_port(int serial_port, char *buffer, int size) {
    int num_bytes = read(serial_port, buffer, size);
    if (num_bytes < 0) {
        printf("Error reading: %s\n", strerror(errno));
        return -1;
    }
    return num_bytes;
}

// For list return, we use pointer
int *read_temp(int servo_count) {
    int *temp_result = (int *)malloc(sizeof(int) * servo_count);
    for (int j = 0; j < servo_count; j++)
        temp_result[j] = 0;

    for (int i = 1; i <= servo_count; i++) {
        int serial_port = open("/dev/ttyUSB0", O_RDWR);
        if (serial_port < 0) {
            printf("Error %i from open: %s\n", errno, strerror(errno));
            free(temp_result);
            return NULL;
        }

        if (configure_serial_port(serial_port) != 0) {
            // Handle configuration error
            close(serial_port);
            free(temp_result);
            return NULL;
        }

        unsigned char checksum = 0;
        unsigned char j = (unsigned char) i;
        unsigned char msg[] = { 0x55, 0x55, j, 0x03, 0x1A, 0x00};

        for (int i = 2; i < 6; i++){
            checksum += msg[i];
        }

        checksum = ~checksum;
        msg[5] = checksum;

        write(serial_port, msg, sizeof(msg));

        char read_buf[7];

        int num_bytes = read_from_serial_port(serial_port, read_buf, sizeof(read_buf));

        if (num_bytes < 0) {
            // Handle read error
            close(serial_port);
            free(temp_result);
            return NULL;
        }

        while (1) {
            if (read_buf[0] == 85 && read_buf[1] == 85 && read_buf[2] == i && read_buf[4] == 26 && read_buf[5] < 100) {
                temp_result[i - 1] = read_buf[5];
                break;
            } else {
                write(serial_port, msg, sizeof(msg));
                num_bytes = read_from_serial_port(serial_port, read_buf, sizeof(read_buf));
            }
        }

        close(serial_port);
    }

    return temp_result;
}




// For list return, we use pointer
unsigned short int *read_angle(int servo_count) {
    unsigned short int *angle_result = (unsigned short int *)malloc(sizeof(unsigned short int) * servo_count);
    for (int j = 0; j < servo_count; j++)
        angle_result[j] = 0;

    for (int i = 1; i <= servo_count; i++) {
        int serial_port = open("/dev/ttyUSB0", O_RDWR);
        if (serial_port < 0) {
            printf("Error %i from open: %s\n", errno, strerror(errno));
            free(angle_result);
            return NULL;
        }

        if (configure_serial_port(serial_port) != 0) {
            // Handle configuration error
            close(serial_port);
            free(angle_result);
            return NULL;
        }

        unsigned char checksum = 0;
        unsigned char j = (unsigned char) i;
        unsigned char msg[] = {0x55, 0x55, j, 0x03, 0x1C, 0x00};

        for (int i = 2; i < 6; i++){
            checksum += msg[i];
        }

        checksum = ~checksum;
        msg[5] = checksum;

        write(serial_port, msg, sizeof(msg));

        char read_buf[8] = {0,};
        int num_bytes = read_from_serial_port(serial_port, read_buf, sizeof(read_buf));

        if (num_bytes < 0) {
            // Handle read error
            close(serial_port);
            free(angle_result);
            return NULL;
        }

        while (1) {
            if (read_buf[0] == 85 && read_buf[1] == 85 && read_buf[2] == i && read_buf[4] == 28 && read_buf[6] < 4) {
                unsigned char result1, result2;
                result1 = read_buf[5];
                result2 = read_buf[6];
                angle_result[i - 1] = result2 * 256 + result1;
                break;
            } else {
                write(serial_port, msg, sizeof(msg));
                num_bytes = read_from_serial_port(serial_port, read_buf, sizeof(read_buf));
            }
        }

        close(serial_port);
    }

    return angle_result;
}



void write_angle(int servo_id, int angle)
{
  int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  if (fd < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
      return;
  }

  if (configure_serial_port(fd) != 0) {
      // Handle configuration error
      close(fd);
      return;
  }
  // There is Maybe an error because it is discarded below the decimal point.
  // angle = angle * 1000 / 240;

  // 3. Find value for first digit. ex) 1000(decimal) -> 3E8(hexa): Find 3 digits
  unsigned char servo_num;
  servo_num = (unsigned char) servo_id;

  int hexa_base = 16;
  int angle_1,angle_2;
  unsigned char angle_low, angle_high;

  angle_1 = angle / (hexa_base * hexa_base);
  angle_2 = angle - angle_1 * (hexa_base * hexa_base);

  angle_low = (unsigned char) angle_2;
  angle_high = (unsigned char) angle_1;

	// 4. Find checksum
	unsigned char checksum = 0;
	unsigned char buffer[10] = {0x55,0x55,servo_num,0x07,0x01,
                                angle_low,angle_high, 0x32, 0x00, 0x00};
	for (int i=2; i<10;i++){
		checksum += buffer[i];
  }
	// 5. Buffer Writing
  checksum = ~ checksum;
  buffer[9] = checksum;

	// 6. Transmission code
  int n = write(fd, buffer, sizeof(buffer));
  if (n != sizeof(buffer)){
    printf("Failed to Send \n");
  }
 close(fd);
}
