#pragma once
#include"SDAStation.h"
#include"lambda.h"
bool KFC(ALLDDOBS* allddobs_before, POSRES* pos_before, RAWDATA* rawdata, ALLSDOBS* allsdaobs, ALLDDOBS* allddobs, Matrix* amb, Matrix* Q_amb);

void Timefixed(ALLDDOBS* allddobs_before, POSRES* pos_before, double& zz_pos, double& zz_dnn, bool yes = true);