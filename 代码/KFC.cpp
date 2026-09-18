#include "KFC.h"


//bool KFC1(RAWDATA* rawdata1, RAWDATA* rawdata)
//{
//    ALLSDOBS allsdaobs1; ALLDDOBS allddobs1;
//    SelectforDD(rawdata1, &allsdaobs1, &allddobs1);
//    int lastsatnum = allddobs1.Num; int lastxnum = lastsatnum * 2 + 3;
//    ALLSDOBS allsdaobs; ALLDDOBS allddobs;
//    if (!SelectforDD(rawdata, &allsdaobs, &allddobs))return false;
//    int nowsatnum = allddobs->Num;
//    Matrix W(lastxnum+4*nowsatnum , 1), P(lastxnum + 4 * nowsatnum, lastxnum + 4 * nowsatnum), B(lastxnum + 4 * nowsatnum, 3 + 2*nowsatnum), X(3 +2* nowsatnum, 1);
//    RTK_LSQ(rawdata1);
//    W = W.leftEmbed(rawdata1->rover.ObsPOS.matrixs.X);
//    P = P.leftEmbed(rawdata1->rover.ObsPOS.matrixs.Q);
//    if(lastsatnum)for (int i = 0; i < 3; i++)B(i, i) = 1;
//    for (int i = 0; i < lastsatnum; i++)
//    {
//        for (int j = 0; j < nowsatnum; j++)
//        {
//            if (!(allddobs1.DDObs[i].System == allddobs->DDObs[j].System))continue; 
//            if (!(allddobs1.DDObs[i].Prn == allddobs->DDObs[j].Prn || allddobs1.DDObs[i].Prn == allddobs->DDObs[j].Prn_ref))continue;
//            if (allddobs->DDObs[j].Prn == allddobs1.DDObs[i].Prn_ref)B(3 + 2 * i, 3 + 2 * j) = -1, B(3 + 2 * i + 1, 3 + 2 * j + 1) = -1;
//            if(allddobs->DDObs[j].Prn==allddobs1.DDObs[i].Prn)B(3 + 2 * i, 3 + 2 * j) = 1, B(3 + 2 * i + 1, 3 + 2 * j + 1) = 1;
//        }
//    }
//    vector<size_t> Bzerorows = B.getZeroRowVectorInRange(0, lastxnum-1);
//    W = W.removeRows(Bzerorows);
//    P = P.removeRows(Bzerorows); P = P.removeCols(Bzerorows);
//    B = B.removeRows(Bzerorows);
//
//    rawdata->rover.KfcPOS = rawdata->rover.ObsPOS;
//    Matrix W2(nowsatnum * 4, 1), B2(nowsatnum * 4, 3 + nowsatnum * 2), C2(nowsatnum * 4, (2 + nowsatnum) * 4), P2(nowsatnum * 4, nowsatnum * 4);
//    double zz_P = 0.3, zz_L = 0.01;
//    int times = 0;
//    do
//    {
//        int k_refer_rover = -1, k_refer_base = -1, k_rover = -1, k_base = -1;
//        double lamb[2] = {};
//        for (int i = 0; i < nowsatnum; i++)
//        {
//            k_rover = allsdaobs->SDSatObs[allddobs->DDObs[i].SD_Index].Raw_rover_Index;
//            k_base = allsdaobs->SDSatObs[allddobs->DDObs[i].SD_Index].Raw_base_Index;
//            k_refer_rover = (allddobs->DDObs[i].System == GPS) ? allsdaobs->SDSatObs[allddobs->GPS_SD_Index].Raw_rover_Index : allsdaobs->SDSatObs[allddobs->BDS_SD_Index].Raw_rover_Index;
//            k_refer_base = (allddobs->DDObs[i].System == GPS) ? allsdaobs->SDSatObs[allddobs->GPS_SD_Index].Raw_base_Index : allsdaobs->SDSatObs[allddobs->BDS_SD_Index].Raw_base_Index;
//            lamb[0] = (allddobs->DDObs[i].System == GPS) ? C_Light / FG1_GPS : C_Light / FG1_BDS;
//            lamb[1] = (allddobs->DDObs[i].System == GPS) ? C_Light / FG2_GPS : C_Light / FG3_BDS;
//
//            double P0_rover_refer = Comdistance(&rawdata->rover.KfcPOS, &rawdata->rover.Obs.SatPVT[k_refer_rover]);
//            double P0_rover_k = Comdistance(&rawdata->rover.KfcPOS, &rawdata->rover.Obs.SatPVT[k_rover]);
//            double dP0_refer = P0_rover_refer - Comdistance(&rawdata->base.ReadPOS, &rawdata->base.Obs.SatPVT[k_refer_base]);
//            double dP0_k = P0_rover_k - Comdistance(&rawdata->base.ReadPOS, &rawdata->base.Obs.SatPVT[k_base]);
//            double ddP0 = dP0_k - dP0_refer;
//            double l = (rawdata->rover.KfcPOS.Pos[0] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[0]) / P0_rover_k - (rawdata->rover.KfcPOS.Pos[0] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[0]) / P0_rover_refer;
//            double m = (rawdata->rover.KfcPOS.Pos[1] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[1]) / P0_rover_k - (rawdata->rover.KfcPOS.Pos[1] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[1]) / P0_rover_refer;
//            double n = (rawdata->rover.KfcPOS.Pos[2] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[2]) / P0_rover_k - (rawdata->rover.KfcPOS.Pos[2] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[2]) / P0_rover_refer;
//            for (int q = 0; q < 4; q++)
//            {
//                B2(i * 4 + q, 0) = l, B2(i * 4 + q, 1) = m, B2(i * 4 + q, 2) = n;
//                if (q == 0 || q == 1)
//                {
//                    W2.row(i * 4 + q) = { allddobs->DDObs[i].ddP[q] - ddP0 };
//                    C2(i * 4 + q, q) = -1 * zz_P, C2(i * 4 + q, (i + 1) * 4 + q) = 1 * zz_P;
//                }
//
//                if (q == 2 || q == 3)
//                {
//                    W2.row(i * 4 + q) = { allddobs->DDObs[i].ddL[q - 2] - ddP0 };
//                    B2(i * 4 + q, 3 + 2 * i + q - 2) = lamb[q - 2];
//                    C2(i * 4 + q, q) = -1 * zz_L, C2(i * 4 + q, (i + 1) * 4 + q) = 1 * zz_L;
//                }
//            }
//        }
//
//        P2 = (C2 * C2.transpose()).inverse();
//        W = W.rightEmbed(W2);
//        P = P.rightEmbed(P2);
//        B = B.rightEmbed(B2);
//        if (!(B.transpose() * P * B).isInvertible())return false;
//        X = (B.transpose() * P * B).inverse() * B.transpose() * P * W;
//        for (int i = 0; i < 3; i++)rawdata->rover.KfcPOS.Pos[i] += X(i, 0);
//        times++;
//    } while (sqrt(X(0, 0) * X(0, 0) + X(1, 0) * X(1, 0) + X(2, 0) * X(2, 0)) > 1e-4 && times < 10);
//    Matrix V = W - B * X;
//    if (!(B.rows() - B.cols() >= 1))return false;
//    rawdata->rover.KfcPOS.SigmaPos = sqrt((V.transpose() * P * V)(0, 0) / (B.rows() - B.cols()));
//    Matrix Q = (B.transpose() * P * B).inverse();
//    rawdata->rover.KfcPOS.PDOP = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
//    rawdata->rover.KfcPOS.matrixs.W = W;
//    rawdata->rover.KfcPOS.matrixs.P = P;
//    rawdata->rover.KfcPOS.matrixs.B = B;
//    rawdata->rover.KfcPOS.matrixs.X = X;
//    rawdata->rover.KfcPOS.matrixs.Q = Q;
//    rawdata->rover.KfcPOS.matrixs.V = V;
//    double distance = sqrt(pow(rawdata->rover.KfcPOS.Pos[0] - rawdata->base.ReadPOS.Pos[0], 2) + pow(rawdata->rover.KfcPOS.Pos[1] - rawdata->base.ReadPOS.Pos[1], 2) + pow(rawdata->rover.KfcPOS.Pos[2] - rawdata->base.ReadPOS.Pos[2], 2));
//    int amb_start = 3;                  // 模糊度参数在X中的起始索引
//    int amb_size = 2 * nowsatnum;          // 模糊度参数总数
//    Matrix amb_(amb_size, 1); rawdata->rover.KfcPOS.matrixs.amb = amb_;
//    for (int i = 0; i < 2 * nowsatnum; i++)(rawdata->rover.KfcPOS.matrixs.amb)(i, 0) = X(i + 3, 0);
//    Matrix Q_amb_(amb_size, amb_size); rawdata->rover.KfcPOS.matrixs.Q_amb = Q_amb_;
//    for (int i = 0; i < amb_size; i++)
//    {
//        for (int j = 0; j < amb_size; j++)
//        {
//            (rawdata->rover.KfcPOS.matrixs.Q_amb)(i, j) = Q(amb_start + i, amb_start + j);
//        }
//    }
//    cout << distance << endl;
//    return true;
//}
//bool KFC1(RAWDATA* rawdata1, RAWDATA* rawdata)
//{
//	return 0;
//}



void Timefixed(ALLDDOBS* allddobs_before, POSRES* pos_before, double& zz_pos, double& zz_dnn,bool yes)
{
    if (!yes)return;
    int satnum = allddobs_before->Num;
    int amb_start = 3;                  // 模糊度参数在X中的起始索引
    int amb_size = 2 * satnum;          // 模糊度参数总数
    Matrix amb(amb_size, 1), Q_amb(amb_size, amb_size);

    for (int i = 0; i < amb_size; i++)amb(i, 0) = pos_before->X(i + 3, 0);
    for (int i = 0; i < amb_size; i++)
    {
        for (int j = 0; j < amb_size; j++)
        {
            Q_amb(i, j) = pos_before->Q(amb_start + i, amb_start + j);
        }
    }
    int m = 2; Matrix F_, s;
    lambda(m, &amb, &Q_amb, &F_, &s);
    if (s(1, 0) / s(0, 0) < 3)
    {
        zz_pos = 1e3;
        zz_dnn = 1e3;
        return;
    }
    for (int i = 0; i < amb_size; i++)pos_before->X(i+3, 0) = F_(i, 0);
    //for (int i = 0; i < amb_size; i++)pos_before->Q(i+3, i+3) = 1e-4;
    zz_dnn = 1e-5;
}
bool KFC(ALLDDOBS* allddobs_before,POSRES *pos_before, RAWDATA* rawdata, ALLSDOBS* allsdaobs, ALLDDOBS* allddobs,Matrix* amb, Matrix* Q_amb)
{
    int num_before = allddobs_before->Num;
    int num = allddobs->Num;
    if(!num_before) //如果上一个历元不存在，则无法时间预测，直接利用观测值rtk结束
    {
        int m = 2; Matrix F_, s;
        if (RTK_LSQ(rawdata, allsdaobs, allddobs, amb, Q_amb))
        {
            RTK_lambdaed(rawdata, allsdaobs, allddobs, amb, Q_amb, 2, &F_, &s);
            rawdata->rover.KfcPOS = rawdata->rover.ObsPOS;
            return true;
        }
        else return false;
    }
    //规定下述中 R为观测误差、 P为观测值权重、Q为估计量误差
	//时间预测 
	Matrix X_time(3+2*num,1), Fy(3+2*num,3+2* num_before),R_noise(3 + 2 * num, 3 + 2 * num), R_time(3 + 2 * num, 3 + 2 * num);
    

    double zz_pos = 10, zz_dnn = 10;//随机误差
    Timefixed(allddobs_before, pos_before, zz_pos,zz_dnn);
    for (int i = 0; i < 3; i++)
    {
        Fy(i, i) = 1;
        R_noise(i, i) = zz_pos * zz_pos;
    }
    for (int i = 0; i <num ; i++)
    {
        bool valid1 = false,valid2 =false;//判断对于本历元的每一个双差，是否能用上一个历元双差进行时间预测（基准星+当前卫星）
        for (int j = 0; j < num_before; j++)
        {
            if (!(allddobs->DDObs[i].System== allddobs_before->DDObs[j].System))continue;

            if (allddobs_before->DDObs[j].Prn == allddobs->DDObs[i].Prn_ref)
            {
                Fy(3 + 2 * i,3 + 2 * j) = -1, Fy(3 + 2 * i + 1, 3 + 2 * j + 1) = -1;
                R_noise(3 + 2 * i, 3 + 2 * i) = zz_dnn * zz_dnn, R_noise(3 + 2 * i + 1, 3 + 2 * i + 1) = zz_dnn * zz_dnn;
                valid1 = true;
            }
            else if(allddobs_before->DDObs[j].Prn_ref == allddobs->DDObs[i].Prn_ref)valid1 = true;
            if (allddobs_before->DDObs[j].Prn == allddobs->DDObs[i].Prn)
            {
                Fy(3 + 2 * i, 3 + 2 * j) = 1, Fy(3 + 2 * i + 1, 3 + 2 * j + 1) = 1;
                R_noise(3 + 2 * i, 3 + 2 * i) = zz_dnn * zz_dnn, R_noise(3 + 2 * i + 1, 3 + 2 * i + 1) = zz_dnn * zz_dnn;
                valid2 = true;
            }
            else if (allddobs_before->DDObs[j].Prn_ref == allddobs->DDObs[i].Prn)valid2=true;
        }
        if (!(valid1 && valid2))//如果不能，则赋予本历元的该双差的时间预测的误差1e10即无限大
        {
            R_noise(3 + 2 * i, 3 + 2 * i) = 1e10 , R_noise(3 + 2 * i + 1, 3 + 2 * i + 1) = 1e10 ;
        }
    }
    X_time = Fy * pos_before->X;
    R_time = Fy * pos_before->Q * Fy.transpose() + R_noise;


    //观测更新
    Matrix W(num * 4, 1), B(num * 4, 3 + num * 2), C_obs(num * 4, (2 + num) * 4), P_obs(num * 4, num * 4), R_obs(num * 4, num * 4);
    double zz_P = 0.3, zz_L = 0.01;
    int k_refer_rover = -1, k_refer_base = -1, k_rover = -1, k_base = -1;
    double lamb[2] = {};
    for (int i = 0; i < num; i++)
    {
        k_rover = allsdaobs->SDSatObs[allddobs->DDObs[i].SD_Index].Raw_rover_Index;
        k_base = allsdaobs->SDSatObs[allddobs->DDObs[i].SD_Index].Raw_base_Index;
        k_refer_rover = (allddobs->DDObs[i].System == GPS) ? allsdaobs->SDSatObs[allddobs->GPS_SD_Index].Raw_rover_Index : allsdaobs->SDSatObs[allddobs->BDS_SD_Index].Raw_rover_Index;
        k_refer_base = (allddobs->DDObs[i].System == GPS) ? allsdaobs->SDSatObs[allddobs->GPS_SD_Index].Raw_base_Index : allsdaobs->SDSatObs[allddobs->BDS_SD_Index].Raw_base_Index;
        lamb[0] = (allddobs->DDObs[i].System == GPS) ? C_Light / FG1_GPS : C_Light / FG1_BDS;
        lamb[1] = (allddobs->DDObs[i].System == GPS) ? C_Light / FG2_GPS : C_Light / FG3_BDS;
        double P0_rover_refer = Comdistance(&rawdata->rover.KfcPOS, &rawdata->rover.Obs.SatPVT[k_refer_rover]);
        double P0_rover_k = Comdistance(&rawdata->rover.KfcPOS, &rawdata->rover.Obs.SatPVT[k_rover]);
        double dP0_refer = P0_rover_refer - Comdistance(&rawdata->base.ReadPOS, &rawdata->base.Obs.SatPVT[k_refer_base]);
        double dP0_k = P0_rover_k - Comdistance(&rawdata->base.ReadPOS, &rawdata->base.Obs.SatPVT[k_base]);
        double ddP0 = dP0_k - dP0_refer;
        double l = (rawdata->rover.KfcPOS.Pos[0] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[0]) / P0_rover_k - (rawdata->rover.KfcPOS.Pos[0] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[0]) / P0_rover_refer;
        double m = (rawdata->rover.KfcPOS.Pos[1] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[1]) / P0_rover_k - (rawdata->rover.KfcPOS.Pos[1] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[1]) / P0_rover_refer;
        double n = (rawdata->rover.KfcPOS.Pos[2] - rawdata->rover.Obs.SatPVT[k_rover].SatPos[2]) / P0_rover_k - (rawdata->rover.KfcPOS.Pos[2] - rawdata->rover.Obs.SatPVT[k_refer_rover].SatPos[2]) / P0_rover_refer;
        for (int q = 0; q < 4; q++)
        {
            B(i * 4 + q, 0) = l, B(i * 4 + q, 1) = m, B(i * 4 + q, 2) = n;
            if (q == 0 || q == 1)
            {
                W.row(i * 4 + q) = { allddobs->DDObs[i].ddP[q]-ddP0+l* rawdata->rover.KfcPOS.Pos[0]+m* rawdata->rover.KfcPOS.Pos[1]+n* rawdata->rover.KfcPOS.Pos[2] };
                C_obs(i * 4 + q, q) = -1 * zz_P, C_obs(i * 4 + q, (i + 1) * 4 + q) = 1 * zz_P;
            }
            if (q == 2 || q == 3)
            {
                W.row(i * 4 + q) = { allddobs->DDObs[i].ddL[q - 2] - ddP0 + l * rawdata->rover.KfcPOS.Pos[0] + m * rawdata->rover.KfcPOS.Pos[1] + n * rawdata->rover.KfcPOS.Pos[2]};
                B(i * 4 + q, 3 + 2 * i + q - 2) = lamb[q - 2];
                C_obs(i * 4 + q, q) = -1 * zz_L, C_obs(i * 4 + q, (i + 1) * 4 + q) = 1 * zz_L;
            }
        }
    }
    R_obs = C_obs * C_obs.transpose();//观测值误差
    P_obs = (C_obs * C_obs.transpose()).inverse();//观测值权重
    

    Matrix K, X(3 + num * 2, 1), R;
    K = R_time * B.transpose() * (B * R_time * B.transpose() + R_obs).inverse();
    X = X_time + K * (W - B * X_time);
    Matrix I; I = K * B; I = I.toIdentity();
    R = (I - K * B) * R_time ;
    //R = (I - K * B) * R_time * (I - K * B).transpose() + K * R_obs * K.transpose();

    for (int m = 0; m < 3; m++)rawdata->rover.KfcPOS.Pos[m] = X(m, 0);
    rawdata->rover.KfcPOS.X = X;
    rawdata->rover.KfcPOS.Q = R;
    //cout << "distance: " << distance << endl;
    //Matrix Q= (Bfixed.transpose() * R * Bfixed).inverse();//估计量误差
    Matrix Q = R;
    Matrix amb_(2 * num, 1), Q_amb_(2 * num, 2 * num);
    *amb = amb_; *Q_amb = Q_amb_;
    for (int i = 0; i < 2 * num; i++)(*amb)(i, 0) = X(i + 3, 0);
    for (int i = 0; i < 2 * num; i++)
    {
        for (int j = 0; j < 2 * num; j++)
        {
            (*Q_amb)(i, j) = Q(3 + i, 3 + j);
        }
    }
    Matrix F_, s;
    lambda(2, amb, Q_amb, &F_, &s);
    for (int k = 0; k < 2 * num; k++)if (fabs((*amb)(k, 0) / F_(k, 0)) > 1.1 || fabs((*amb)(k, 0) / F_(k, 0)) < 0.9) 
    {
        RTK_lambdaed(rawdata, allsdaobs, allddobs, amb, Q_amb ,2, &F_, &s); return true;
    }
	return true;
}
