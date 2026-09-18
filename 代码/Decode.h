#pragma once
#include"allnecehead.h"
#include"CoordinateConvert.h"
unsigned int UI4(unsigned char* p);
int I4(unsigned char* p);
short I2(unsigned char* p);
float F4(unsigned char* p);
double D8(unsigned char* p);
unsigned short UI2(unsigned char* p);
unsigned int Crc32(unsigned char* buff, int len);

void DecodeRange(unsigned char* data, int len, EPOCHOBSDATA* obs);
void DecodeGpsEph(unsigned char* data, int len, GPSEPHREC geph[]);
void DecodeBdsEph(unsigned char* data, int len, GPSEPHREC beph[]);
void DecodePsrPos(unsigned char* data, int len, POSRES* pos);
bool DecodeNovOem7Dat(vector<unsigned char> &buff_all,int &len_used, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos);
