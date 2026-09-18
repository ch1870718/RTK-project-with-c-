#include"SPP.h"

void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double UserPos[3])//功能:Epoch->SatObs计算Epoch->SatPVT
{
    //memset(Epoch->SatPVT, 0, sizeof(Epoch->SatPVT));
    for (int i = 0; i < Epoch->SatNum; i++)
    {
        Epoch->SatPVT[i].Prn = Epoch->SatObs[i].Prn;
        Epoch->SatPVT[i].System = Epoch->SatObs[i].System;
        //1.先剔除无效观测
        if (Epoch->SatObs[i].Valid == false)
        {
            Epoch->SatPVT[i].Valid = false;
            continue;
        }
        //2.计算发射时刻和其位置速度
        //2.1GPS
        if (Epoch->SatObs[i].System == GPS)
        {
            //2.1.1发射时间
            int prn = Epoch->SatObs[i].Prn;
            double PIF = Epoch->ComObs[i].PIF;
            GPSEPHREC* GPSeph = GPSEph + prn - 1;
            GPSTIME t_rec = Epoch->Time;//接收时间
            GPSTIME t_trans;//发射时间
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light;t_trans.Week = t_rec.Week;
            CompSatClkOff(prn, GPS, &t_trans, GPSEph, BDSEph, &Epoch->SatPVT[i]);
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light - Epoch->SatPVT[i].SatClkOft;
            CompSatClkOff(prn, GPS, &t_trans, GPSEph, BDSEph, &Epoch->SatPVT[i]);
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light - Epoch->SatPVT[i].SatClkOft;
            if (!CompSatClkOff(prn, GPS, &t_trans, GPSEph, BDSEph, &Epoch->SatPVT[i])){Epoch->SatPVT[i].Valid = false;continue;}
            //2.2.2发射时的位置速度
            CompGPSSatPVT(prn, &t_trans, GPSeph, &(Epoch->SatPVT[i]));
            EarthRotate(UserPos, GPSeph, &Epoch->SatPVT[i]);
        }
        //2.2BDS
        if (Epoch->SatObs[i].System == BDS)
        {
            //2.2.1发射时间
            int prn = Epoch->SatObs[i].Prn;
            double PIF = Epoch->ComObs[i].PIF;
            GPSEPHREC* BDSeph = BDSEph + prn - 1;
            GPSTIME t_rec = Epoch->Time;//接收时间
            GPSTIME t_trans;//发射时间
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light; t_trans.Week = t_rec.Week;
            CompSatClkOff(prn, BDS, &t_trans, GPSEph, BDSEph, &Epoch->SatPVT[i]);
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light - Epoch->SatPVT[i].SatClkOft;
            CompSatClkOff(prn, BDS, &t_trans, GPSEph, BDSEph, &Epoch->SatPVT[i]);
            t_trans.SecOfWeek = t_rec.SecOfWeek - PIF / C_Light - Epoch->SatPVT[i].SatClkOft;
            if (!CompSatClkOff(prn, BDS, &t_trans, GPSEph, BDSEph, &Epoch->SatPVT[i])){Epoch->SatPVT[i].Valid = false;continue;}
            //2.2.2发射时的位置速度
            CompBDSSatPVT(prn, &t_trans, BDSeph, &(Epoch->SatPVT[i]));
            EarthRotate(UserPos, BDSeph, &Epoch->SatPVT[i]);
        }
    }
}
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res)//功能:由Epoch->Sat最小二乘计算得到Res->Pos
{
    //最小二乘计算接收机位置的前置准备
    Matrix B(MAXCHANNUM, 5); Matrix W(MAXCHANNUM, 1);
    //迭代的最小二乘计算过程
        //迭代原因：其最小二乘计算的矩阵由现有的卫星信息决定，而最小二乘计算的结果即接收机信息又可以使得卫星信息更加准确
    Matrix dX(3, 1); dX = { {Res->Pos[0]},{Res->Pos[1]},{Res->Pos[2]} }; Matrix dX_prev(3, 1);
    bool have_BDS = false, have_GPS = false;double gap = 999.0;
    do 
    {
        //初始化/更新各要素
            //注意一定要初始化矩阵大小，因为经过一次迭代后的接收机位置再用于计算卫星信息，其卫星信息有效性可能由无效转为有效则原来矩阵大小小于所需矩阵大小
        Matrix dX_prev = dX;
        double UserPos[3] = { 0 }; for (int k = 0; k < 3; k++)UserPos[k] = dX(k, 0);
        Matrix C(MAXCHANNUM, 5); Matrix D(MAXCHANNUM, 1);B = C; W = D;
        have_BDS = false, have_GPS = false;
        ComputeSatPVTAtSignalTrans(Epoch, GPSEph, BDSEph, UserPos);
        //填充最小二乘的矩阵
        for (int i = 0; i < Epoch->SatNum; i++)
        {
            if (Epoch->SatPVT[i].Valid == false)continue;
            double SatPos[3] = { 0 }; for (int k=0; k < 3; k++) SatPos[k] = Epoch->SatPVT[i].SatPos[k];
            double Rou = sqrt((SatPos[0] - UserPos[0]) * (SatPos[0] - UserPos[0]) + (SatPos[1] - UserPos[1]) * (SatPos[1] - UserPos[1]) + (SatPos[2] - UserPos[2]) * (SatPos[2] - UserPos[2]));
            double l = -(SatPos[0] - UserPos[0]) / Rou, m = -(SatPos[1] - UserPos[1]) / Rou, n = -(SatPos[2] - UserPos[2]) / Rou;
            double tgd = 0.0; 
            if (Epoch->SatObs[i].System == GPS)
            {
                B.row(i) = { l,m,n,1,0 };
                tgd = 0.0;
                have_GPS = true;
            }
            if (Epoch->SatObs[i].System == BDS)
            {
                B.row(i) = { l,m,n,0,1 };
                GPSEPHREC* BDSeph = BDSEph + Epoch->SatObs[i].Prn - 1;
                tgd = C_Light * BDSeph->TGD1 * FG1_BDS * FG1_BDS / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
                have_BDS = true;
            }     
            W.row(i) = {Epoch->ComObs[i].PIF - Rou + l * UserPos[0] + m * UserPos[1] + n * UserPos[2] + Epoch->SatPVT[i].SatClkOft * C_Light - Epoch->SatPVT[i].TropCorr - tgd };
        }

        B = B.removeZeroRowsAndCols(); 
        W = W.removeZeroRows();
        if (!(B.transpose() * B).isInvertible())return false;
        dX = (B.transpose() * B).inverse() * B.transpose() * W;

        //跳出迭代的条件因子
        gap = (dX(0, 0) - dX_prev(0, 0)) * (dX(0, 0) - dX_prev(0, 0)) + (dX(1, 0) - dX_prev(1, 0)) * (dX(1, 0) - dX_prev(1, 0)) + (dX(2, 0) - dX_prev(2, 0)) * (dX(2, 0) - dX_prev(2, 0));
    } while (fabs(gap)>1e-4);
    //将最小二乘计算的结果转化为所需接收机信息
    for (int k = 0; k < 3; k++)Res->Pos[k] = dX(k,0);
    if(have_BDS && have_GPS)Res->GPS_clockBias = dX(3, 0), Res->BDS_clockBias = dX(4, 0);else if(have_GPS)Res->GPS_clockBias = dX(3, 0);else if(have_BDS)Res->BDS_clockBias = dX(3, 0);
    Matrix Q = (B.transpose() * B).inverse();Res->PDOP = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
    Matrix V = B * dX - W;if(B.rows() - B.cols()>=1)Res->SigmaPos = sqrt((V.transpose() * V)(0, 0) / (B.rows() - B.cols()));
    
    return true;
}
void SPV(EPOCHOBSDATA* Epoch, POSRES* Res)//功能:Epoch->Sat和Res->Pos最小二乘计算得到Res->Vel
{
    //最小二乘计算接收机位置的前置准备
    Matrix B(MAXCHANNUM, 4); Matrix W(MAXCHANNUM, 1);
    //填充最小二乘的矩阵
    for (int i = 0; i < Epoch->SatNum; i++)
    {
        if (Epoch->SatPVT[i].Valid == false)continue;
        double SatPos[3] = { 0 }; double SatVel[3] = { 0 }; double ResPos[3] = { 0 }; 
        for (int k = 0; k < 3; k++) SatPos[k] = Epoch->SatPVT[i].SatPos[k],SatVel[k] = Epoch->SatPVT[i].SatVel[k],ResPos[k] = Res->Pos[k];
        double Rou = sqrt((SatPos[0] - ResPos[0]) * (SatPos[0] - ResPos[0]) + (SatPos[1] - ResPos[1]) * (SatPos[1] - ResPos[1]) + (SatPos[2] - ResPos[2]) * (SatPos[2] - ResPos[2]));
        double l = -(SatPos[0] - ResPos[0]) / Rou, m = -(SatPos[1] - ResPos[1]) / Rou, n = -(SatPos[2] - ResPos[2]) / Rou;
        B.row(i) = { l,m,n,1 };
        W.row(i) = { Epoch->SatObs[i].D[0] - ((SatPos[0] - ResPos[0]) * SatVel[0] + (SatPos[1] - ResPos[1]) * SatVel[1] + (SatPos[2] - ResPos[2]) * SatVel[2]) / Rou + C_Light * Epoch->SatPVT[i].SatClkSft };
    }
    B=B.removeZeroRows(); W=W.removeZeroRows();if (B.rows() < 4)return;
    //将最小二乘计算的结果转化为所需接收机信息
    Matrix dX = (B.transpose() * B).inverse() * B.transpose() * W;for (int k = 0; k < 3; k++)Res->Vel[k] = dX(k,0);
    Matrix V = B * dX - W;Res->SigmaVel = sqrt((V.transpose() * V)(0, 0) / (B.rows() - B.cols()));
    
    
}


