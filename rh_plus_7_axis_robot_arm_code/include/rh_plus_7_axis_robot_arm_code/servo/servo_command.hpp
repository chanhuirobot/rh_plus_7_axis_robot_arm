#ifndef SERVO_READ_DATA_HPP_
#define SERVO_READ_DATA_HPP_

int * read_temp(int servo_count);
unsigned short int * read_angle(int servo_count);
void write_angle(int servo_id, int angle);

#endif
