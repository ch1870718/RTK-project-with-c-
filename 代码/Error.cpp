#include"Error.h"
double Hopfield(const double H, const double Elev) //功能：计算对流层延迟
{
    // 检查输入高度是否在有效范围内（0-30km）
    if (H < 0 || H > 3e4) return 0;

    // 计算当前高度的气象参数
    const double T = T0_STD - 0.0065 * (H - H0_STD);           // 当前温度(K)
    const double RH = RH0_STD * exp(-0.0006396 * (H - H0_STD)); // 当前相对湿度
    const double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T); // 水汽压
    const double p = P0_STD * pow(1 - 0.0000226 * (H - H0_STD), 5.225); // 当前气压

    // 计算干分量尺度高度
    const double hd = HD_FACTOR1 + HD_FACTOR2 * (T0_STD - 273.16);
    // 计算干湿分量延迟
    const double Kw = 155.2E-7 * 4810 * e * (HW_STD - H) / (T * T);
    const double Kd = 155.2E-7 * p * (hd - H) / T0_STD;

    // 计算仰角相关的对流层总延迟
    const double zenithAngleCorrection = sqrt(Elev * Elev + 6.25) * PI / 180.0;
    return (Kd + Kw) / sin(zenithAngleCorrection);
}
void DetectOutlier(EPOCHOBSDATA* Obs) //功能：Obs->SatObs得到的Obs->ComObs和历史的Obs->ComObs共同评判obs有效性、平滑Obs->ComObs                                  
{
    MWGF CurComObs[MAXCHANNUM]; // 存储当前历元组合观测值，初始化为0
    double f1 = 0.0, f2 = 0.0;        // 信号频率参数

    //遍历所有卫星观测数据
    for (int i = 0; i < Obs->SatNum; i++)
    {
        CurComObs[i].Prn = Obs->SatObs[i].Prn; CurComObs[i].Sys = Obs->SatObs[i].System;
      //1.检查观测值有效性（跳过无效值）
        //检查locktime有效性(rtk的新增)
        if (Obs->SatObs[i].Valid == false)continue;
        
        //检查parity（rtk的新增）
        if (Obs->SatObs[i].half[0] == 0|| Obs->SatObs[i].half[1] == 0) { Obs->SatObs[i].Valid = false; continue; }
        //检查观测数据存在为0的情况
        if (fabs(Obs->SatObs[i].P[0]) < 1e-5 || fabs(Obs->SatObs[i].P[1]) < 1e-5 || fabs(Obs->SatObs[i].L[0]) < 1e-5 || fabs(Obs->SatObs[i].L[1]) < 1e-5)
        {
            Obs->SatObs[i].Valid = false; continue;
        }
      //2.检查组合观测值有效性（与历史值不能偏差太大）
         //2.1 根据卫星系统设置信号频率
        if (CurComObs[i].Sys == GPS) { f1 = FG1_GPS; f2 = FG2_GPS; }
        else if (CurComObs[i].Sys == BDS) { f1 = FG1_BDS; f2 = FG3_BDS; }
         //2.2 计算组合观测值
        CurComObs[i].n = 1;
        CurComObs[i].MW = (Obs->SatObs[i].L[0] * f1 - Obs->SatObs[i].L[1] * f2) / (f1 - f2) - (Obs->SatObs[i].P[0] * f1 + Obs->SatObs[i].P[1] * f2) / (f1 + f2);
        CurComObs[i].GF = Obs->SatObs[i].L[0] - Obs->SatObs[i].L[1];
        CurComObs[i].PIF = (f1 * f1 * Obs->SatObs[i].P[0] - f2 * f2 * Obs->SatObs[i].P[1]) / (f1 * f1 - f2 * f2);
         //2.3与历史组合观测值比较
        if (Obs->ComObs[i].n >= 1)
        {
            for (int j = 0; j < MAXCHANNUM; j++)
            {
                if (CurComObs[i].Prn == Obs->ComObs[j].Prn && CurComObs[i].Sys == Obs->ComObs[j].Sys)
                { 
                    double dmw = fabs(CurComObs[i].MW - Obs->ComObs[j].MW); double dgf = fabs(CurComObs[i].GF - Obs->ComObs[j].GF);
                    // 差异在阈值内，计算即将更新的组合观测值
                    if (dmw < 3.0 && dgf < 0.05)
                    {
                        CurComObs[i].n += Obs->ComObs[j].n;
                        CurComObs[i].MW = (CurComObs[i].MW + Obs->ComObs[j].MW * (CurComObs[i].n - 1)) / CurComObs[i].n;
                        CurComObs[i].PIF = (f1 * f1 * Obs->SatObs[i].P[0] - f2 * f2 * Obs->SatObs[i].P[1]) / (f1 * f1 - f2 * f2);
                        

                    }
                    else { Obs->SatObs[i].Valid = false; }  // 差异超出阈值，标记为异常
                    break;
                }
            }

        }
        
    }    
        //2.5更新所有组合观测值
    for (int i = 0; i < MAXCHANNUM; i++) { Obs->ComObs[i] = CurComObs[i]; Obs->ComObs[i].n = max(Obs->ComObs[i].n,CurComObs[i].n ); }
}

