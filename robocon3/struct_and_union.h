#ifndef __STRUCT_AND_UNION_H__
#define __STRUCT_AND_UNION_H__

typedef struct r_packet_t Receive_Packet;
typedef struct t_packet_t Transmit_Packet;

struct r_packet_t{
  int signaltype;
  long unique_id;
  byte rot_dir;
  byte datatype;
  float data[4];
  Receive_Packet *next;
};

struct t_packet_t{
  int signaltype;
  long unique_id;
  float data[9];
  Transmit_Packet *next;
};

union byte2int {
  byte b[2];
  int integar;
};

union byte2long {
  byte b[4];
  long number;
};

union byte2float {
  byte b[4];
  float decimal;
};

void pinAssign();

void IMUcheck();

void LTsetup();

void receivePacket();

void work();

void serialTransmit(Transmit_Packet t_packet);

int get_temp();

float get_dist(float temperature);

#endif
