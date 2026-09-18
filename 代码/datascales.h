#pragma once
#pragma once
#include"const.h"

/* 卫星导航系统定义 */
enum GNSSsys { GPS = 1, BDS };
/* 解码数据源类型定义 */
enum readmode{file=1,ip};
/* 站类型定义 */
enum Stationtype {Base=0,Rover};
/* 坐标系统定义 */
union XYZ    // 地心地固坐标系
{
	struct {
		double x;
		double y;
		double z;
	};
	double xyz[3];
	XYZ():x(0.0),y(0.0),z(0.0){}

};

union GEOCOOR    // 大地坐标系
{
	struct {
		double longitude;   // 经度
		double latitude;    // 纬度
		double height;      // 高程
	};
	double blh[3];
	GEOCOOR() : longitude(0.0), latitude(0.0), height(0.0) {}
};

union ENUCOOR    // 测站地平坐标系
{
	struct {
		double east;   // 东
		double north;  // 北
		double up;     // 天
	};
	double enu[3];
	ENUCOOR() : east(0.0), north(0.0), up(0.0) {}
};

/* 时间系统定义 */
struct COMMONTIME   // 普通公历时间
{
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;
	COMMONTIME() : Year(0), Month(0), Day(0), Hour(0), Minute(0), Second(0.0) {}
};

struct MJDTIME    // 简化儒略日（MJD）从1858年11月17日子夜开始
{
	int Days;
	double FracDay;
	MJDTIME() : Days(0), FracDay(0.0) {}
};

struct GPSTIME   // GPS时间
{
	unsigned short Week;
	double SecOfWeek;
	GPSTIME() : Week(0), SecOfWeek(0.0) {}
};

/* 观测数据相关结构体 */
struct SATOBSDATA    // 卫星观测值数据结构体
{
	short Prn;
	GNSSsys System;
	double P[2], L[2], D[2];  // 伪距、载波相位、多普勒频移
	double cn0[2], LockTime[2];  // 信噪比、锁定时间
	unsigned char half[2];  // 半周模糊度标志
	bool Valid;
	SATOBSDATA()
	{
		Prn = 0;
		System = GPS;
		for (int i = 0; i < 2; i++)
		{
			P[i] = 0;
			L[i] = 0;
			D[i] = 0;
			cn0[i] = 0;
			LockTime[i] = 0;
			half[i] = 0;
		}
		Valid = true;

	}
};

struct MWGF   //MW和GF两种组合观测值结构体
{
	short Prn;
	GNSSsys Sys;
	double MW;   // Melbourne-Wübbena组合
	double GF;   // Geometry-Free组合
	double PIF;  // 相位电离层残差
	int n;       // 更新次数
	MWGF()
	{
		Prn = 0;
		Sys = GPS;
		MW = 0;
		GF = 0;
		PIF = 0;
		n = 0;
	}
};

struct SATPVT    // 卫星位置速度时间数据结构体
{
	short Prn;
	GNSSsys System;
	double SatPos[3], SatVel[3];  // 卫星位置和速度
	double SatClkOft, SatClkSft;  // 卫星钟差和钟漂
	double Elevation, Azimuth;    // 卫星仰角和方位角
	double TropCorr;              // 对流层改正
	double Tgd1, Tgd2;            // 群延迟改正
	bool Valid;
	SATPVT()
	{
		Prn = 0;
		System = GPS;
		for (int i = 0; i < 3; i++)
		{
			SatPos[i] = 1e-8;
			SatVel[i] = 0;
		}
		Elevation = PI / 2;
		Azimuth = 0;
		SatClkOft = SatClkSft = 0;
		Tgd1 = Tgd2 = TropCorr = 0;
		Valid = true;
	}
};

struct GPSEPHREC    // 星历数据结构体
{
	short PRN;
	GNSSsys System;
	GPSTIME TOC, TOE;  // 卫星时钟参考时间、星历参考时间
	double ClkBias, ClkDrift, ClkDriftRate;  // 卫星钟差参数
	double IODE, IODC;  // 星历数据龄期、时钟数据龄期
	double SqrtA, M0, e, OMEGA, i0, omega;  // 轨道参数
	double Crs, Cuc, Cus, Cic, Cis, Crc;  // 摄动参数
	double DeltaN, OMEGADot, iDot;  // 轨道摄动参数
	int SVHealth;  // 卫星健康状态
	double TGD1, TGD2;  // 群延迟改正
	GPSEPHREC()
	{
		PRN = 0;
		System = GPS;
		IODC = IODE = 0;
		ClkBias = ClkDrift = ClkDriftRate = 0;
		SqrtA = M0 = e = OMEGA = i0 = omega = 0;
		Crs = Cuc = Cus = Cic = Cis = Crc = 0;
		DeltaN = OMEGADot = iDot = 0;
		SVHealth = 0;
		TGD1 = TGD2 = 0;
	}
};

/* 定位结果相关结构体 */
struct POSRES	// 接收机定位结果数据结构体
{
	GPSTIME Time;
	double Pos[3], Vel[3];  // 位置和速度
	double PDOP, SigmaPos, SigmaVel;  // 精度因子和标准差
	double BDS_clockBias, GPS_clockBias;  // 系统间钟差
	int SatNum;  // 参与定位的卫星数
	Matrix X, Q;//定位结果矩阵，定位结果的误差矩阵
	POSRES()
	{
		for (int i = 0; i < 3; i++)
		{
			Pos[i] = 1e-4;
			Vel[i] = 0;
		}
		PDOP = SigmaPos = SigmaVel = 0;
		BDS_clockBias = GPS_clockBias = 0;
		SatNum = 0;
	}
};
struct EPOCHOBSDATA    // 观测值数据结构体
{
	GPSTIME Time;
	short SatNum;
	SATOBSDATA SatObs[MAXCHANNUM];  // 卫星观测值
	MWGF ComObs[MAXCHANNUM];        // 组合观测值
	SATPVT SatPVT[MAXCHANNUM];    // 卫星位置速度时间
	EPOCHOBSDATA()
	{
		SatNum = 0;
	}
};



//单站的数据
struct StationDATA
{
	/*GPSTIME Time;*/
	EPOCHOBSDATA Obs;
	GPSEPHREC GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM];
	POSRES ReadPOS;
	POSRES ObsPOS;
	POSRES KfcPOS;
};
//站组的数据
struct RAWDATA
{
	StationDATA base;
	StationDATA rover;
};


//单卫星单差数据
struct SDSATOBS
{
	int Raw_rover_Index;
	int Raw_base_Index;
	short Prn;
	GNSSsys System;
	double dP[2], dL[2];  // 伪距、载波相位单差
	bool Valid;
	SDSATOBS()
	{
		Raw_rover_Index = -1;//单向索引 可不可以单差的[Prn,System]整体作为索引来索引Rawdata的其中一个站的[Prn,System]对应的Obs  可以，但索引更便利后续
		Raw_base_Index = -1;
		Prn = 0;
		System = GPS;
		dP[0] = dP[1] = 0.0; dL[0] = dL[1] = 0.0;
		Valid = true;
	}
};
struct SDMWGF
{
	short Prn;
	GNSSsys System;
	double dMW;   // Melbourne-Wübbena组合
	double dGF;   // Geometry-Free组合
	double dPIF;  // 相位电离层残差
	int n;       // 更新次数
	SDMWGF()
	{
		Prn = 0;
		System = GPS;
		dMW = 0.0;
		dGF = 0.0;
		dPIF = 0.0;
		n = 0;
	}

};
//卫星组单差数据
struct ALLSDOBS
{
	GPSTIME Time;
	int GPSNum, BDSNum;
	int Num;
	SDSATOBS SDSatObs[MAXCHANNUM];
	SDMWGF SDComObs[MAXCHANNUM];
	ALLSDOBS()
	{
		GPSNum = BDSNum = 0;
		Num = 0;
	}
};
//单卫星双差数据
struct DDOBS
{
	GNSSsys System;
	short Prn;
	int SD_Index;

	short Prn_ref;
	
	double ddP[2], ddL[2];
	DDOBS()
	{
		Prn = 0;
		System = GPS;
		Prn_ref = 0;
		SD_Index = -1;
		for (int i = 0; i < 2; i++)
		{
			ddP[i] = 0.0; ddL[i] = 0.0;
		}
	}
};
//卫星组双差数据
struct ALLDDOBS
{
	GPSTIME Time;
	short GPS_Prn, BDS_Prn;
	int GPS_SD_Index, BDS_SD_Index;
	int GPSNum, BDSNum;
	int Num;
	DDOBS DDObs[MAXGPSNUM+ MAXBDSNUM - 2];
	ALLDDOBS()
	{
		GPS_Prn = BDS_Prn = -1;
		GPS_SD_Index= BDS_SD_Index = -1;
		GPSNum= BDSNum=0;
		Num = 0;
	}
};

