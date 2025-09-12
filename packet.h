ifndef PACKET_H
define PACKET_H

struct {

  int rpm;
  int distance;
  int speed;
  float current;

} motordrivepacket

void sendtoSPI(struct mydatapacket) {
  
}



endif