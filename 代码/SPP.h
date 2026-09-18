#pragma once
#include"allnecehead.h"
#include"SatPos.h"

void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA* Epoch, GPSEPHREC* Eph, GPSEPHREC* BDSEph, double UserPos[3]);
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res);
void SPV(EPOCHOBSDATA* Epoch, POSRES* Res);
