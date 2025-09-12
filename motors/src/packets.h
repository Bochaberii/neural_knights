#ifndef PACKET_H
#define PACKET_H

struct MotorDrivePacket
{
    int rpm;
    float distance;
    int speed;
    float current;
};

void sendtoSPI(const MotorDrivePacket &packet);

#endif
