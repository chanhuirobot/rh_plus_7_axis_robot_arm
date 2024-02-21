#ifndef SERVO_READ_DATA_HPP_
#define SERVO_READ_DATA_HPP_

int configure_serial_port(int serial_port);
int read_from_serial_port(int serial_port, char *buffer, int size);
int * read_temp(int servo_count);
unsigned short int * read_angle(int servo_count);
void write_angle(int servo_id, int angle);

#endif
