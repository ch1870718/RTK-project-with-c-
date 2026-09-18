#include "SDAStation.h"
bool TimeSyn_data(FILE* FObs_rover, FILE* FObs_base, RAWDATA* rawdata, double dt)
{
    static int len_used_rover = MAXRAWLEN; static int len_used_base = MAXRAWLEN;
    static vector<unsigned char> buff_rover(MAXRAWLEN); static vector<unsigned char>buff_base(MAXRAWLEN);
    while (!feof(FObs_rover))
    {
        if (fread(buff_rover.data() + MAXRAWLEN - len_used_rover, sizeof(unsigned char), len_used_rover, FObs_rover) < len_used_rover)return false;
        if(DecodeNovOem7Dat(buff_rover, len_used_rover, &rawdata->rover.Obs, rawdata->rover.GpsEph, rawdata->rover.BdsEph, &rawdata->rover.ReadPOS))break;
    }
    double t_rover = rawdata->rover.Obs.Time.Week * 604800 + rawdata->rover.Obs.Time.SecOfWeek;
    static double t_base = rawdata->base.Obs.Time.Week * 604800 + rawdata->base.Obs.Time.SecOfWeek;
    if (fabs(t_rover - t_base) < dt)return true;
    while (!feof(FObs_base))
    {
        if (fread(buff_base.data() + MAXRAWLEN - len_used_base, sizeof(unsigned char), len_used_base, FObs_base) < len_used_base)return false;
        if (DecodeNovOem7Dat(buff_base, len_used_base, &rawdata->base.Obs, rawdata->base.GpsEph, rawdata->base.BdsEph, &rawdata->base.ReadPOS))
        {
            t_base = rawdata->base.Obs.Time.Week * 604800 + rawdata->base.Obs.Time.SecOfWeek;
            if (fabs(t_rover - t_base) < dt)return true;
            else if (t_rover < t_base + dt)return false;
        }
    }
}
bool TimeSyn_ip(SOCKET* NetGps1, SOCKET* NetGps2, RAWDATA* rawdata, double dt)
{

    static int len_used_rover = MAXRAWLEN; static  int len_used_base = MAXRAWLEN;
    static vector<unsigned char> buff_rover(MAXRAWLEN); static vector<unsigned char>buff_base(MAXRAWLEN);
    
    if (recv(*NetGps1, reinterpret_cast<char*>(buff_rover.data()) + MAXRAWLEN - len_used_rover, len_used_rover, 0) <= 0)return false;
    DecodeNovOem7Dat(buff_rover, len_used_rover, &rawdata->rover.Obs, rawdata->rover.GpsEph, rawdata->rover.BdsEph, &rawdata->rover.ReadPOS);
    if (recv(*NetGps2, reinterpret_cast<char*>(buff_base.data()) + MAXRAWLEN - len_used_base, len_used_base, 0) <= 0)return false;
    DecodeNovOem7Dat(buff_base, len_used_base, &rawdata->base.Obs, rawdata->base.GpsEph, rawdata->base.BdsEph, &rawdata->base.ReadPOS);

    double t_rover = rawdata->rover.Obs.Time.Week * 604800 + rawdata->rover.Obs.Time.SecOfWeek;
    double t_base = rawdata->base.Obs.Time.Week * 604800 + rawdata->base.Obs.Time.SecOfWeek;
    double dt_now = t_rover - t_base;
    /*static int times = 0;
    times++;
    dt = exp(10 - times) + 0.5;*/
    if (fabs(dt_now) < dt)return true;
    else return false;
}

void SD(RAWDATA* rawdata, ALLSDOBS *allsdaobs)
{
    allsdaobs->Num = 0;
    allsdaobs->GPSNum = 0;
    allsdaobs->BDSNum = 0;
    allsdaobs->Time = rawdata->rover.Obs.Time;
	for (int i = 0; i < rawdata->rover.Obs.SatNum;i++)
	{
		if (!rawdata->rover.Obs.SatPVT[i].Valid)continue;
        if (rawdata->rover.Obs.SatPVT[i].Elevation < 32.0 / 180.0* PI)continue;
		bool find = false; int k = 0;
		for (; k < rawdata->base.Obs.SatNum; k++)
            if (rawdata->base.Obs.SatObs[k].Prn == rawdata->rover.Obs.SatObs[i].Prn&& rawdata->base.Obs.SatObs[k].System== rawdata->rover.Obs.SatObs[i].System) 
            {find = true; break;}
		if (!find)continue;
        if (!rawdata->base.Obs.SatPVT[k].Valid)continue;
        if (rawdata->rover.Obs.SatObs[i].System == GPS)allsdaobs->GPSNum++;
        else if (rawdata->rover.Obs.SatObs[i].System == BDS)allsdaobs->BDSNum++;
        allsdaobs->SDSatObs[allsdaobs->Num].Raw_rover_Index = i;
        allsdaobs->SDSatObs[allsdaobs->Num].Raw_base_Index = k;
		allsdaobs->SDSatObs[allsdaobs->Num].Prn = rawdata->rover.Obs.SatObs[i].Prn;
		allsdaobs->SDSatObs[allsdaobs->Num].System = rawdata->rover.Obs.SatObs[i].System;
		allsdaobs->SDSatObs[allsdaobs->Num].dL[0] = rawdata->rover.Obs.SatObs[i].L[0]+ rawdata->rover.Obs.SatPVT[i].SatClkOft - (rawdata->base.Obs.SatObs[k].L[0]+ rawdata->base.Obs.SatPVT[k].SatClkOft);
		allsdaobs->SDSatObs[allsdaobs->Num].dL[1] = rawdata->rover.Obs.SatObs[i].L[1] + rawdata->rover.Obs.SatPVT[i].SatClkOft - (rawdata->base.Obs.SatObs[k].L[1] + rawdata->base.Obs.SatPVT[k].SatClkOft);
		allsdaobs->SDSatObs[allsdaobs->Num].dP[0] = rawdata->rover.Obs.SatObs[i].P[0] + rawdata->rover.Obs.SatPVT[i].SatClkOft - (rawdata->base.Obs.SatObs[k].P[0] + rawdata->base.Obs.SatPVT[k].SatClkOft);
		allsdaobs->SDSatObs[allsdaobs->Num].dP[1] = rawdata->rover.Obs.SatObs[i].P[1] + rawdata->rover.Obs.SatPVT[i].SatClkOft - (rawdata->base.Obs.SatObs[k].P[1] + rawdata->base.Obs.SatPVT[k].SatClkOft);
		allsdaobs->Num++;

	}
	allsdaobs->Time = rawdata->rover.Obs.Time;
}
void Detect(ALLSDOBS* allsdaobs)                                   
{
    SDMWGF SDComObs[MAXCHANNUM]; // 存储当前历元组合观测值，初始化为0
    double f1 = 0.0, f2 = 0.0;        // 信号频率参数
    //遍历所有卫星观测数据
    for (int i = 0; i < allsdaobs->Num; i++)
    {
        SDComObs[i].Prn = allsdaobs->SDSatObs[i].Prn; SDComObs[i].System = allsdaobs->SDSatObs[i].System;
        if (SDComObs[i].System == GPS) { f1 = FG1_GPS; f2 = FG2_GPS; }
        else if (SDComObs[i].System == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; }
        SDComObs[i].n = 1;
        SDComObs[i].dMW = (allsdaobs->SDSatObs[i].dL[0] * f1 - allsdaobs->SDSatObs[i].dL[1] * f2) / (f1 - f2) - (allsdaobs->SDSatObs[i].dP[0] * f1 + allsdaobs->SDSatObs[i].dP[1] * f2) / (f1 + f2);
        SDComObs[i].dGF = allsdaobs->SDSatObs[i].dL[0] - allsdaobs->SDSatObs[i].dL[1];
        SDComObs[i].dPIF = (f1 * f1 * allsdaobs->SDSatObs[i].dP[0] - f2 * f2 * allsdaobs->SDSatObs[i].dP[1]) / (f1 * f1 - f2 * f2);
        if (allsdaobs->SDComObs[i].n>= 1)
        {
            for (int j = 0; j < allsdaobs->Num; j++)
            {
                if (SDComObs[i].Prn == allsdaobs->SDComObs[j].Prn && SDComObs[i].System == allsdaobs->SDComObs[j].System)
                {
                    double dmw = fabs(SDComObs[i].dMW - allsdaobs->SDComObs[j].dMW); double dgf = fabs(SDComObs[i].dGF - allsdaobs->SDComObs[j].dGF);
                    if (dmw < 3.0 && dgf < 0.05)
                    {
                        SDComObs[i].n += allsdaobs->SDComObs[j].n;
                        SDComObs[i].dMW = (SDComObs[i].dMW + allsdaobs->SDComObs[j].dMW * (SDComObs[i].n - 1)) / SDComObs[i].n;
                        SDComObs[i].dGF = (SDComObs[i].dGF + allsdaobs->SDComObs[j].dGF * (SDComObs[i].n - 1)) / SDComObs[i].n;
                        SDComObs[i].dPIF = (f1 * f1 * allsdaobs->SDSatObs[i].dP[0] - f2 * f2 * allsdaobs->SDSatObs[i].dP[1]) / (f1 * f1 - f2 * f2);

                    }
                    else { allsdaobs->SDSatObs[i].Valid = false; } 
                    break;
                }
            }

        }
    }
    //2.4更新所有组合观测值
    for (int i = 0; i < MAXCHANNUM; i++) { allsdaobs->SDComObs[i] = SDComObs[i]; }
}
bool SelectforDD(RAWDATA* rawdata, ALLSDOBS* allsdaobs, ALLDDOBS* allddobs)
{
    allddobs->Num = 0;
    allddobs->GPSNum = 0;
    allddobs->BDSNum = 0;
    allddobs->GPS_Prn = -1;
    allddobs->BDS_Prn = -1;
    allddobs->GPS_SD_Index = -1;
    allddobs->BDS_SD_Index = -1;
    allddobs->Time = allsdaobs->Time;
    allddobs->Time.Week = allsdaobs->Time.Week; allddobs->Time.SecOfWeek = allsdaobs->Time.SecOfWeek;
    double GPS_judge = 0.0, BDS_judge = 0.0;
    for (int i = 0; i < allsdaobs->Num; i++)
    {
        if (!allsdaobs->SDSatObs[i].Valid)continue;
        int m = allsdaobs->SDSatObs[i].Raw_rover_Index;
        int n = allsdaobs->SDSatObs[i].Raw_base_Index;
        if (rawdata->rover.Obs.SatObs[m].LockTime[0] < 6 || rawdata->rover.Obs.SatObs[m].LockTime[1] < 6)continue;
        if (rawdata->base.Obs.SatObs[n].LockTime[0] < 6 || rawdata->base.Obs.SatObs[n].LockTime[1] < 6)continue;
        double judge = rawdata->rover.Obs.SatObs[m].cn0[0] + rawdata->rover.Obs.SatObs[m].cn0[1] +
            rawdata->base.Obs.SatObs[n].cn0[0] + rawdata->base.Obs.SatObs[n].cn0[1] +
            rawdata->rover.Obs.SatPVT[m].Elevation * 180 / PI + rawdata->base.Obs.SatPVT[n].Elevation * 180 / PI;

        if (rawdata->rover.Obs.SatObs[m].System == GPS && judge > GPS_judge)
        {
            GPS_judge = judge;
            allddobs->GPS_Prn = allsdaobs->SDSatObs[i].Prn;
            allddobs->GPS_SD_Index = i;
        }
        else if (rawdata->rover.Obs.SatObs[m].System == BDS && judge > BDS_judge)
        {
            BDS_judge = judge;
            allddobs->BDS_Prn = allsdaobs->SDSatObs[i].Prn;
            allddobs->BDS_SD_Index = i;
        }
    }
    if (allddobs->GPS_SD_Index == -1 && allddobs->BDS_SD_Index == -1)return false;

    for (int i = 0; i < allsdaobs->Num; i++)
    {
        int i_refer =(allsdaobs->SDSatObs[i].System == GPS)? allddobs->GPS_SD_Index: allddobs->BDS_SD_Index;
        int Prn_refer= (allsdaobs->SDSatObs[i].System == GPS) ? allddobs->GPS_Prn : allddobs->BDS_Prn;
        if (allsdaobs->SDSatObs[i].Prn == Prn_refer)continue;
        if (i_refer == -1)continue;
        if (allsdaobs->SDSatObs[i].System == GPS)allddobs->GPSNum++;
        else if (allsdaobs->SDSatObs[i].System == BDS)allddobs->BDSNum++;
        allddobs->DDObs[allddobs->Num].Prn = allsdaobs->SDSatObs[i].Prn;
        allddobs->DDObs[allddobs->Num].System= allsdaobs->SDSatObs[i].System;
        allddobs->DDObs[allddobs->Num].Prn_ref = allsdaobs->SDSatObs[i].Prn;
        allddobs->DDObs[allddobs->Num].SD_Index = i;
        allddobs->DDObs[allddobs->Num].ddP[0] = allsdaobs->SDSatObs[i].dP[0] - allsdaobs->SDSatObs[i_refer].dP[0];
        allddobs->DDObs[allddobs->Num].ddP[1] = allsdaobs->SDSatObs[i].dP[1] - allsdaobs->SDSatObs[i_refer].dP[1];
        allddobs->DDObs[allddobs->Num].ddL[0] = allsdaobs->SDSatObs[i].dL[0] - allsdaobs->SDSatObs[i_refer].dL[0];
        allddobs->DDObs[allddobs->Num].ddL[1] = allsdaobs->SDSatObs[i].dL[1] - allsdaobs->SDSatObs[i_refer].dL[1];
        allddobs->Num++;
    } 
    return true;
}
double Comdistance(POSRES* Posres, SATPVT* satpvt)
{
    return sqrt(
        (Posres->Pos[0] - satpvt->SatPos[0]) * (Posres->Pos[0] - satpvt->SatPos[0])
        + (Posres->Pos[1] - satpvt->SatPos[1]) * (Posres->Pos[1] - satpvt->SatPos[1])
        + (Posres->Pos[2] - satpvt->SatPos[2]) * (Posres->Pos[2] - satpvt->SatPos[2]));
}
bool RTK_LSQ(RAWDATA* rawdata, ALLSDOBS *allsdaobs,ALLDDOBS *allddobs,Matrix* amb,Matrix* Q_amb)
{
    if (allddobs->Num <= 3)return false;
    int satnum = allddobs->Num;
    Matrix W(satnum * 4, 1), B(satnum * 4, 3 + satnum * 2), X(3 + satnum * 2, 1), C(satnum * 4, (2 + satnum) * 4), P(satnum * 4, satnum * 4);
    double zz_P = 0.3, zz_L = 0.01;
    int times = 0;
    do
    {
        int k_refer_rover = -1, k_refer_base = -1, k_rover = -1, k_base = -1;
        double lamb[2] = {};
        for (int i = 0; i < satnum; i++)
        {
            k_rover = allsdaobs->SDSatObs[allddobs->DDObs[i].SD_Index].Raw_rover_Index;
            k_base = allsdaobs->SDSatObs[allddobs->DDObs[i].SD_Index].Raw_base_Index;
            k_refer_rover = (allddobs->DDObs[i].System==GPS)?allsdaobs->SDSatObs[allddobs->GPS_SD_Index].Raw_rover_Index: allsdaobs->SDSatObs[allddobs->BDS_SD_Index].Raw_rover_Index;
            k_refer_base = (allddobs->DDObs[i].System == GPS) ? allsdaobs->SDSatObs[allddobs->GPS_SD_Index].Raw_base_Index : allsdaobs->SDSatObs[allddobs->BDS_SD_Index].Raw_base_Index;
            lamb[0] = (allddobs->DDObs[i].System == GPS)?C_Light / FG1_GPS: C_Light / FG1_BDS;
            lamb[1] = (allddobs->DDObs[i].System == GPS) ? C_Light / FG2_GPS: C_Light / FG3_BDS;

            double P0_rover_refer = Comdistance(&rawdata->rover.ObsPOS, &rawdata->rover.Obs.SatPVT[k_refer_rover]);
            double P0_rover_k = Comdistance(&rawdata->rover.ObsPOS, &rawdata->rover.Obs.SatPVT[k_rover]);
            double dP0_refer = P0_rover_refer - Comdistance(&rawdata->base.ReadPOS, &rawdata->base.Obs.SatPVT[k_refer_base]);
            double dP0_k = P0_rover_k - Comdistance(&rawdata->base.ReadPOS, &rawdata->base.Obs.SatPVT[k_base]);
            double ddP0 = dP0_k - dP0_refer;
            double l = (rawdata->rover.ObsPOS.Pos[0] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[0]) / P0_rover_k - (rawdata->rover.ObsPOS.Pos[0] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[0]) / P0_rover_refer;
            double m = (rawdata->rover.ObsPOS.Pos[1] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[1]) / P0_rover_k - (rawdata->rover.ObsPOS.Pos[1] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[1]) / P0_rover_refer;
            double n = (rawdata->rover.ObsPOS.Pos[2] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[2]) / P0_rover_k - (rawdata->rover.ObsPOS.Pos[2] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[2]) / P0_rover_refer;
            for (int q = 0; q < 4; q++)
            {
                B(i * 4 + q, 0) = l, B(i * 4 + q, 1) = m, B(i * 4 + q, 2) = n;
                if (q == 0 || q == 1)
                {
                    W.row(i * 4 + q) = { allddobs->DDObs[i].ddP[q] - ddP0 };
                    C(i * 4 + q, q) = -1 * zz_P, C(i * 4 + q, (i + 1) * 4 + q) = 1 * zz_P;
                }

                if (q == 2 || q == 3)
                {
                    W.row(i * 4 + q) = { allddobs->DDObs[i].ddL[q - 2] - ddP0 };
                    B(i * 4 + q, 3 + 2 * i + q - 2) = lamb[q - 2];
                    C(i * 4 + q, q) = -1 * zz_L, C(i * 4 + q, (i + 1) * 4 + q) = 1 * zz_L;
                }
            }
        }
        P = (C * C.transpose()).inverse();
        if (!(B.transpose() * P * B).isInvertible())return false;
        X = (B.transpose() * P * B).inverse() * B.transpose() * P * W;
        for (int i = 0; i < 3; i++)rawdata->rover.ObsPOS.Pos[i] += X(i, 0);
        
        times++;
    } while (sqrt(X(0, 0) * X(0, 0) + X(1, 0) * X(1, 0) + X(2, 0) * X(2, 0)) > 1e-4 && times < 10);
    Matrix V = W - B * X;
    if (!(B.rows() - B.cols() >= 1))return false;
    rawdata->rover.ObsPOS.SigmaPos = sqrt((V.transpose() * P * V)(0, 0) / (B.rows() - B.cols()));
    Matrix Q = (B.transpose() * P * B).inverse();
    rawdata->rover.ObsPOS.PDOP = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
    rawdata->rover.ObsPOS.X = X;
    for (int i = 0; i < 3; i++)rawdata->rover.ObsPOS.X(i, 0) += rawdata->rover.ObsPOS.Pos[i];
    rawdata->rover.ObsPOS.Q = Q;
    //double distance = sqrt(pow(rawdata->rover.ObsPOS.Pos[0] - rawdata->base.ReadPOS.Pos[0], 2) + pow(rawdata->rover.ObsPOS.Pos[1] - rawdata->base.ReadPOS.Pos[1], 2) + pow(rawdata->rover.ObsPOS.Pos[2] - rawdata->base.ReadPOS.Pos[2], 2));
    int amb_start = 3;                  // 模糊度参数在X中的起始索引
    int amb_size = 2 * satnum;          // 模糊度参数总数
    Matrix amb_(amb_size, 1), Q_amb_(amb_size, amb_size);
    *amb = amb_; *Q_amb = Q_amb_;
    for (int i = 0; i < 2 * satnum; i++)(*amb)(i, 0) = X(i + 3, 0);
    for (int i = 0; i < amb_size; i++) 
    {
        for (int j = 0; j < amb_size; j++) 
        {
            (*Q_amb)(i, j) = Q(amb_start + i, amb_start + j);
        }
    }
    //cout << distance << endl;
    return true;
}
bool RTK_lambdaed(RAWDATA* rawdata, ALLSDOBS* allsdaobs, ALLDDOBS* allddobs, Matrix* amb, Matrix* Q_amb,int m,Matrix* F_,Matrix* s)
{
    if (allddobs->Num <= 3)return false;
    int satnum = allddobs->Num;
    if (!RTK_LSQ(rawdata, allsdaobs, allddobs,amb, Q_amb))return false;
    if (m <= 0) { cout << "m需要大于等于1" << endl; return false; }
    lambda(m,amb, Q_amb, F_, s);
    if ((*s)(1, 0) / (*s)(0, 0) < 3)return true;
    Matrix W(satnum * 4, 1), B(satnum * 4, 3 ), X(3 , 1), C(satnum * 4, (2 + satnum) * 4), P(satnum * 4, satnum * 4);
    double zz_P = 0.3, zz_L = 0.01;
    int times = 0;
    do
    {
        int k_refer_rover = -1, k_refer_base = -1, k_rover = -1, k_base = -1;
        double lamb[2] = {};
        for (int i = 0; i < satnum; i++)
        {
            k_rover = allsdaobs->SDSatObs[allddobs->DDObs[i].SD_Index].Raw_rover_Index;
            k_base = allsdaobs->SDSatObs[allddobs->DDObs[i].SD_Index].Raw_base_Index;
            k_refer_rover = (allddobs->DDObs[i].System == GPS) ? allsdaobs->SDSatObs[allddobs->GPS_SD_Index].Raw_rover_Index : allsdaobs->SDSatObs[allddobs->BDS_SD_Index].Raw_rover_Index;
            k_refer_base = (allddobs->DDObs[i].System == GPS) ? allsdaobs->SDSatObs[allddobs->GPS_SD_Index].Raw_base_Index : allsdaobs->SDSatObs[allddobs->BDS_SD_Index].Raw_base_Index;
            lamb[0] = (allddobs->DDObs[i].System == GPS) ? C_Light / FG1_GPS : C_Light / FG1_BDS;
            lamb[1] = (allddobs->DDObs[i].System == GPS) ? C_Light / FG2_GPS : C_Light / FG3_BDS;

            double P0_rover_refer = Comdistance(&rawdata->rover.ObsPOS, &rawdata->rover.Obs.SatPVT[k_refer_rover]);
            double P0_rover_k = Comdistance(&rawdata->rover.ObsPOS, &rawdata->rover.Obs.SatPVT[k_rover]);
            double dP0_refer = P0_rover_refer - Comdistance(&rawdata->base.ReadPOS, &rawdata->base.Obs.SatPVT[k_refer_base]);
            double dP0_k = P0_rover_k - Comdistance(&rawdata->base.ReadPOS, &rawdata->base.Obs.SatPVT[k_base]);
            double ddP0 = dP0_k - dP0_refer;
            double l = (rawdata->rover.ObsPOS.Pos[0] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[0]) / P0_rover_k - (rawdata->rover.ObsPOS.Pos[0] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[0]) / P0_rover_refer;
            double m = (rawdata->rover.ObsPOS.Pos[1] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[1]) / P0_rover_k - (rawdata->rover.ObsPOS.Pos[1] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[1]) / P0_rover_refer;
            double n = (rawdata->rover.ObsPOS.Pos[2] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[2]) / P0_rover_k - (rawdata->rover.ObsPOS.Pos[2] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[2]) / P0_rover_refer;
            for (int q = 0; q < 4; q++)
            {
                B(i * 4 + q, 0) = l, B(i * 4 + q, 1) = m, B(i * 4 + q, 2) = n;
                if (q == 0 || q == 1)
                {
                    W.row(i * 4 + q) = { allddobs->DDObs[i].ddP[q] - ddP0 };
                    C(i * 4 + q, q) = -1 * zz_P, C(i * 4 + q, (i + 1) * 4 + q) = 1 * zz_P;
                }

                if (q == 2 || q == 3)
                {
                    W.row(i * 4 + q) = { allddobs->DDObs[i].ddL[q - 2] - ddP0-(*F_)(2*i+q-2,0)*lamb[q-2]};
                    C(i * 4 + q, q) = -1 * zz_L, C(i * 4 + q, (i + 1) * 4 + q) = 1 * zz_L;
                }
            }
        }

        P = (C * C.transpose()).inverse();
        if (!(B.transpose() * P * B).isInvertible())return false;
        X = (B.transpose() * P * B).inverse() * B.transpose() * P * W;
        for (int i = 0; i < 3; i++)rawdata->rover.ObsPOS.Pos[i] += X(i, 0);

        times++;
    } while (sqrt(X(0, 0) * X(0, 0) + X(1, 0) * X(1, 0) + X(2, 0) * X(2, 0)) > 1e-4 && times < 10);
    Matrix V = W - B * X;
    if (!(B.rows() - B.cols() >= 1))return false;
    rawdata->rover.ObsPOS.SigmaPos = sqrt((V.transpose() * P * V)(0, 0) / (B.rows() - B.cols()));
    Matrix Q = (B.transpose() * P * B).inverse();
    rawdata->rover.ObsPOS.PDOP = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));

    return true;
}
