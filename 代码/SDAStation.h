#pragma once
#include"allnecehead.h"
#include"Decode.h"
#include"Error.h"
#include"SPP.h"
#include"lambda.h"
bool TimeSyn_data(FILE* FObs_base, FILE* FObs_rover, RAWDATA* rawdata, double dt = 0.5);
bool TimeSyn_ip(SOCKET* NetGps1, SOCKET* NetGps2, RAWDATA* rawdata, double dt = 0.5);
void SD(RAWDATA* rawdata, ALLSDOBS *allsdaobs);
void Detect(ALLSDOBS* allsdaobs);
bool SelectforDD(RAWDATA* rawdata, ALLSDOBS* allsdaobs, ALLDDOBS* allddobs);
double Comdistance(POSRES* Posres, SATPVT* satpvt);
bool RTK_LSQ(RAWDATA* rawdata,  ALLSDOBS* allsdaobs, ALLDDOBS* allddobs, Matrix* amb, Matrix* Q_amb);
bool RTK_lambdaed(RAWDATA* rawdata, ALLSDOBS* allsdaobs, ALLDDOBS* allddobs, Matrix* amb, Matrix* Q_amb, int m, Matrix* F_, Matrix* s);
