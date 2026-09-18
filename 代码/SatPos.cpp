#include "SatPos.h"
#include"Error.h"
#include "matrix.h"

bool CompSatClkOff(const int Prn, const GNSSsys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid)//功能：同系统的卫星群 判决星历过期+计算钟差
{
    //规范化两种卫星系统的时间参数
    GPSEPHREC* eph; GPSTIME CurT= *t; double LimT;
    if (Sys == GPS){eph = GPSEph + Prn - 1; LimT = 7500.0;}
    else if (Sys == BDS) { eph = BDSEph + Prn - 1; LimT = 3900.0; CurT.Week -= 1356; CurT.SecOfWeek -= 14; }
    else return false;
    //判决未过期的星历并钟差模型计算其钟差
                //星历过期指钟差模型过期，即当前时刻-卫星时钟参考时间（钟差模型制定时刻）超过了阈值
    double dt = (CurT.Week - eph->TOC.Week) * 604800.0 + CurT.SecOfWeek - eph->TOC.SecOfWeek;
    if (fabs(dt) > LimT || eph->SVHealth != 0)return false;
    Mid->SatClkOft = eph->ClkBias + eph->ClkDrift * dt + eph->ClkDriftRate * pow(dt, 2);
    Mid->SatClkSft = eph->ClkDrift + 2.0 * eph->ClkDriftRate * dt;
    return true;
}
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* GPSEph, SATPVT* Mid) //功能：星历计算GPS系统的卫星群信息
{
    // 1.轨道参数
    const double A = GPSEph->SqrtA * GPSEph->SqrtA;
    const double n0 = sqrt(GM_GPS) / pow(GPSEph->SqrtA, 3);
    const double tk = t->SecOfWeek - GPSEph->TOE.SecOfWeek;
    const double n = GPSEph->DeltaN + n0;

    // 2.开普勒方程迭代求解偏近点角Ek
    double Mk = GPSEph->M0 + n * tk;
    double Ek = Mk;double Ek_prev;
    do { Ek_prev = Ek; Ek = Mk+ GPSEph->e * sin(Ek); } while (fabs(Ek - Ek_prev) > 1e-12);
    
    //3.位置
      // 3.1位置中间量
    // 轨道平面内参数计算
    const double vk = atan2(sqrt(1 - GPSEph->e * GPSEph->e) * sin(Ek), cos(Ek) - GPSEph->e);
    const double Phik = GPSEph->omega + vk;
    // 摄动修正项
    const double delta_uk = GPSEph->Cuc * cos(2 * Phik) + GPSEph->Cus * sin(2 * Phik);
    const double delta_rk = GPSEph->Crs * sin(2 * Phik) + GPSEph->Crc * cos(2 * Phik);
    const double delta_ik = GPSEph->Cic * cos(2 * Phik) + GPSEph->Cis * sin(2 * Phik);
    // 卫星在轨道平面内的位置
    const double uk = Phik + delta_uk;
    const double rk = A * (1 - GPSEph->e * cos(Ek)) + delta_rk;
    const double ik = GPSEph->i0 + delta_ik + GPSEph->iDot * tk;
    // 升交点赤经
    const double OMEGAk = GPSEph->OMEGA +(GPSEph->OMEGADot - OMEGAE_GPS) * tk -OMEGAE_GPS * GPSEph->TOE.SecOfWeek;
    // 轨道平面坐标转换为ECEF坐标
    const double xk = rk * cos(uk);
    const double yk = rk * sin(uk);
      //3.2位置
    Mid->SatPos[0] = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
    Mid->SatPos[1] = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
    Mid->SatPos[2] = yk * sin(ik);

    

    //4.速度
    const double EkDot = n / (1 - GPSEph->e * cos(Ek));
    const double PhikDot = sqrt(1 - GPSEph->e * GPSEph->e) * EkDot / (1 - GPSEph->e * cos(Ek));
    const double ukDot = PhikDot + 2 * (-GPSEph->Cuc * sin(2 * Phik) + GPSEph->Cus * cos(2 * Phik)) * PhikDot;
    const double rkDot = A * GPSEph->e * sin(Ek) * EkDot + 2 * (GPSEph->Crs * cos(2 * Phik) - GPSEph->Crc * sin(2 * Phik)) * PhikDot;
    const double ikDot = GPSEph->iDot + 2 * (-GPSEph->Cic * sin(2 * Phik) + GPSEph->Cis * cos(2 * Phik)) * PhikDot;
    const double OMEGAkDot = GPSEph->OMEGADot - OMEGAE_GPS;
    const double xkDot = rkDot * cos(uk) - rk * sin(uk) * ukDot;
    const double ykDot = rkDot * sin(uk) + rk * cos(uk) * ukDot;
    Matrix RDot(3, 4);RDot = { {cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk * sin(OMEGAk) + yk * cos(OMEGAk) * cos(ik)), yk * sin(OMEGAk) * sin(ik)},
                               {sin(OMEGAk), cos(OMEGAk) * cos(ik), xk * cos(OMEGAk) - yk * sin(OMEGAk) * cos(ik), -yk * cos(OMEGAk) * sin(ik)},
                               {0, sin(ik), 0, yk * cos(ik)}};
    Matrix V(4, 1); V= { {xkDot}, {ykDot}, {OMEGAkDot}, {ikDot} };
    Matrix X(3, 1); X = RDot * V;
    Mid->SatVel[0] = X(0,0);
    Mid->SatVel[1] = X(1,0);
    Mid->SatVel[2] = X(2,0);

    
    //5.钟差（包含相对论效应）
    const double delta_tr = F * GPSEph->e * GPSEph->SqrtA * sin(Ek);
    const double dt_clk = t->SecOfWeek - GPSEph->TOC.SecOfWeek;
    const double delta_trDot = F * GPSEph->e * GPSEph->SqrtA * cos(Ek) * EkDot;
    Mid->SatClkOft = GPSEph->ClkBias + GPSEph->ClkDrift * dt_clk + GPSEph->ClkDriftRate * dt_clk * dt_clk + delta_tr;
    Mid->SatClkSft = GPSEph->ClkDrift + 2 * GPSEph->ClkDriftRate * dt_clk + delta_trDot;

    //6.群延迟参数
    Mid->Tgd1 = GPSEph->TGD1;

    return 0;
}
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* BDSEph, SATPVT* Mid) //功能：星历计算BDS系统的卫星群信息
{
    // 1. 时间系统转换 (GPS → BDT)
    GPSTIME bdt;
    bdt.SecOfWeek = t->SecOfWeek - GPST_BDT_SEC;
    bdt.Week = t->Week - GPST_BDT_WEEK;

    // 2. 轨道参数
    double A = BDSEph->SqrtA * BDSEph->SqrtA;           // 轨道长半轴
    double n0 = sqrt(GM_BDS) / pow(BDSEph->SqrtA, 3);   // 平均运动率
    double tk = bdt.SecOfWeek - BDSEph->TOE.SecOfWeek; // 时间差
    double n = BDSEph->DeltaN + n0;                      // 修正后平均运动率

    // 3. 开普勒方程迭代求解偏近点角Ek
    double Mk = BDSEph->M0 + n * tk;
    double Ek = Mk; double Ek_prev;
    do {Ek_prev = Ek; Ek = Mk + BDSEph->e * sin(Ek);} while (fabs(Ek - Ek_prev) >1e-12);

    // 4. 位置中间量
    double vk = atan2(sqrt(1 - BDSEph->e * BDSEph->e) * sin(Ek), cos(Ek) - BDSEph->e); // 真近点角
    double Phik = BDSEph->omega + vk;                     // 纬度幅角
    double delta_uk = BDSEph->Cuc * cos(2 * Phik) + BDSEph->Cus * sin(2 * Phik); // 纬度摄动
    double delta_rk = BDSEph->Crs * sin(2 * Phik) + BDSEph->Crc * cos(2 * Phik); // 半径摄动
    double delta_ik = BDSEph->Cic * cos(2 * Phik) + BDSEph->Cis * sin(2 * Phik); // 倾角摄动

    double uk = Phik + delta_uk;           // 修正后纬度幅角
    double rk = A * (1 - BDSEph->e * cos(Ek)) + delta_rk; // 修正后轨道半径
    double ik = BDSEph->i0 + delta_ik + BDSEph->iDot * tk; // 修正后轨道倾角
    double xk = rk * cos(uk),yk = rk * sin(uk); // 轨道平面内笛卡尔坐标

    // 5. 速度中间量
    double EkDot = n / (1 - BDSEph->e * cos(Ek));
    double PhikDot = sqrt(1 - BDSEph->e * BDSEph->e) * EkDot / (1 - BDSEph->e * cos(Ek));
    double ukDot = PhikDot + 2 * (-BDSEph->Cuc * sin(2 * Phik) + BDSEph->Cus * cos(2 * Phik)) * PhikDot;
    double rkDot = A * BDSEph->e * sin(Ek) * EkDot + 2 * (BDSEph->Crs * cos(2 * Phik) - BDSEph->Crc * sin(2 * Phik)) * PhikDot;
    double ikDot = BDSEph->iDot + 2 * (-BDSEph->Cic * sin(2 * Phik) + BDSEph->Cis * cos(2 * Phik)) * PhikDot;
    double xkDot = rkDot * cos(uk) - rk * sin(uk) * ukDot;
    double ykDot = rkDot * sin(uk) + rk * cos(uk) * ukDot;

    // 6. 卫星分类求位置、速度
    double OMEGAk, OMEGAkDot;
    if (Prn <= 5 || (Prn >= 59 && Prn <= 63)) // GEO/IGSO卫星特殊处理
    { 
        // 轨道平面坐标
        OMEGAk = BDSEph->OMEGA + BDSEph->OMEGADot * tk - OMEGAE_BDS * BDSEph->TOE.SecOfWeek;
        OMEGAkDot = BDSEph->OMEGADot;
        double xgk = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
        double ygk = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
        double zgk = yk * sin(ik);
        // 北斗GEO卫星特殊旋转 (X轴旋转5°)
        Matrix Rz(3, 3);Rz = { {cos(OMEGAE_BDS * tk),sin(OMEGAE_BDS * tk),0},{-sin(OMEGAE_BDS * tk),cos(OMEGAE_BDS * tk),0},{0,0,1} };
        Matrix Rx(3, 3);Rx = { {1,0,0},{0,cos(-PI / 36),sin(-PI / 36)},{0,-sin(-PI / 36),cos(-PI / 36)} };
        Matrix Rotate(3,3); Rotate = Rz * Rx;
        Matrix Xgk(3, 1);Xgk = { {xgk},{ygk},{zgk} };
        Matrix Pos(3, 1); Pos = Rotate * Xgk;
        //位置
        Mid->SatPos[0] = Pos(0,0);
        Mid->SatPos[1] = Pos(1,0);
        Mid->SatPos[2] = Pos(2,0);

        // 速度中间量 (考虑地球自转)
        Matrix RDot(3, 4);RDot = { {cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk * sin(OMEGAk) + yk * cos(OMEGAk) * cos(ik)), yk * sin(OMEGAk) * sin(ik)},
                                    {sin(OMEGAk), cos(OMEGAk) * cos(ik), xk * cos(OMEGAk) - yk * sin(OMEGAk) * cos(ik), -yk * cos(OMEGAk) * sin(ik)},
                                    {0, sin(ik), 0, yk * cos(ik)} };
        Matrix V(4, 1);V = { {xkDot},{ykDot},{OMEGAkDot},{ikDot} };
        Matrix XgkDot(3, 1);XgkDot = RDot * V;
        Matrix RzDot(3, 3); RzDot = { {-OMEGAE_BDS * sin(OMEGAE_BDS * tk),OMEGAE_BDS * cos(OMEGAE_BDS * tk),0},
                                      {-OMEGAE_BDS * cos(OMEGAE_BDS * tk),-OMEGAE_BDS * sin(OMEGAE_BDS * tk),0},
                                      {0,0,0} };
        Matrix M1(3, 1); M1 = Rotate * XgkDot;
        Matrix RzDotMRx(3, 3); RzDotMRx = RzDot * Rx;
        Matrix M2(3, 1); M2 = RzDotMRx * Xgk;
        Matrix Vel(3, 1); Vel = M1 +M2;
        //速度
        Mid->SatVel[0] = Vel(0,0); 
        Mid->SatVel[1] = Vel(1,0);
        Mid->SatVel[2] = Vel(2,0);
    }
    else { // MEO卫星标准处理

        OMEGAk = BDSEph->OMEGA + (BDSEph->OMEGADot - OMEGAE_BDS) * tk - OMEGAE_BDS * BDSEph->TOE.SecOfWeek;
        OMEGAkDot = BDSEph->OMEGADot - OMEGAE_BDS;

        // 直接计算ECEF坐标
        Mid->SatPos[0] = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
        Mid->SatPos[1] = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
        Mid->SatPos[2] = yk * sin(ik);

        // 速度计算
        Matrix RDot(3, 4); RDot = { {cos(OMEGAk), -sin(OMEGAk) * cos(ik), -(xk * sin(OMEGAk) + yk * cos(OMEGAk) * cos(ik)), yk * sin(OMEGAk) * sin(ik)},
                                    {sin(OMEGAk), cos(OMEGAk) * cos(ik), xk * cos(OMEGAk) - yk * sin(OMEGAk) * cos(ik), -yk * cos(OMEGAk) * sin(ik)},
                                    {0, sin(ik), 0, yk * cos(ik)} };
        Matrix V(4, 1); V = { {xkDot},{ykDot},{OMEGAkDot},{ikDot} };
        Matrix X(3, 1); X = RDot * V;

        Mid->SatVel[0] = X(0,0);
        Mid->SatVel[1] = X(1,0);
        Mid->SatVel[2] = X(2,0);
    }

    // 7. 卫星钟差计算 (含相对论效应)
    double dt_clk = bdt.SecOfWeek - BDSEph->TOC.SecOfWeek;
    double delta_tr = F * BDSEph->e * sin(Ek) * BDSEph->SqrtA;     
    double delta_trDot = F * EkDot * cos(Ek) * BDSEph->SqrtA * BDSEph->e;

    Mid->SatClkOft = BDSEph->ClkBias + BDSEph->ClkDrift * dt_clk + BDSEph->ClkDriftRate * dt_clk * dt_clk + delta_tr;
    Mid->SatClkSft = BDSEph->ClkDrift + 2 * BDSEph->ClkDriftRate * dt_clk + delta_trDot;

    // 8. 群延迟参数
    Mid->Tgd1 = BDSEph->TGD1;
    Mid->Tgd2 = BDSEph->TGD2;

    return 0;
}
int EarthRotate(double UserPos[3], const GPSEPHREC* Eph, SATPVT* Mid)// 功能：针对单颗卫星 地球自转校正 + 对流层延迟计算
{
    // 1. 参数初始化（根据卫星系统选择地球模型）
    double omegae = (Eph->System == GPS) ? OMEGAE_GPS : OMEGAE_BDS;  // 地球自转角速度
    double R = (Eph->System == GPS) ? R_CGS2K : R_WGS84;              // 地球半径
    double E = (Eph->System == GPS) ? E_CGS2K : E_WGS84;              // 地球扁率

    // 2. 地球自转修正（Z轴旋转）
    double dx = Mid->SatPos[0] - UserPos[0];
    double dy = Mid->SatPos[1] - UserPos[1];
    double dz = Mid->SatPos[2] - UserPos[2];
    double delta_t = sqrt(dx * dx + dy * dy + dz * dz) / C_Light;
    double theta = omegae * delta_t;
    Matrix Rz(3, 3); Rz = { {cos(theta), sin(theta), 0},
                            {-sin(theta), cos(theta), 0},
                            {0, 0, 1}};
    Matrix Pos(3, 1); Pos = { {Mid->SatPos[0]},{Mid->SatPos[1]},{Mid->SatPos[2]} };
    Matrix Vel(3, 1);Vel= { {Mid->SatVel[0]},{Mid->SatVel[1]},{Mid->SatVel[2]} };

    Matrix tempPos(3, 1); tempPos = Rz * Pos;
    Matrix tempVel(3, 1); tempVel = Rz * Vel;

    // 3.更新卫星位置和速度
    for (int k = 0; k < 3; k++) 
    { Mid->SatPos[k] = tempPos(k, 0); Mid->SatVel[k] = tempVel(k, 0); }
    // 4. 计算卫星仰角和方位角
    CompSatElAz(UserPos, Mid->SatPos, &Mid->Elevation, &Mid->Azimuth, R, E);

    // 5. 对流层延迟校正（Hopfield模型）
    GEOCOOR UserGeo;
    XYZ xyz;xyz.x = UserPos[0], xyz.y = UserPos[1], xyz.z = UserPos[2];
    XYZToBLH(xyz, UserGeo, R, E);
    double elevation_deg = Mid->Elevation * 180 / PI;
    Mid->TropCorr = Hopfield(UserGeo.height, elevation_deg);

    return 0;
}


