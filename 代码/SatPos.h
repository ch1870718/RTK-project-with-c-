#pragma once
#include"allnecehead.h"
#include"CoordinateConvert.h"
bool CompSatClkOff(const int Prn, const GNSSsys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid);
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* GPSEph, SATPVT* Mid);
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* BDSEph, SATPVT* Mid);
int EarthRotate(double UserPos[3], const GPSEPHREC* Eph, SATPVT* Mid);