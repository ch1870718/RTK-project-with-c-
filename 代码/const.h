#pragma once
#include<cmath>

/* 地球模型参数 */
#define R_WGS84  6378137.0          /* 地球半径 [m]; WGS-84 */
#define F_WGS84  1.0/298.257223563  /* 扁率; WGS-84 */
#define E_WGS84  (F_WGS84 * (2 - F_WGS84)) /* 偏心率平方 */
#define Omega_WGS 7.2921151467e-5   /* 地球自转角速度 [rad/s]; WGS-84 */
#define GM_Earth   398600.5e+9      /* 地球引力常数 [m^3/s^2]; WGS-84 */

#define R_CGS2K  6378137.0          /* 地球半径 [m]; CGCS2000 */
#define F_CGS2K  1.0/298.257222101  /* 扁率; CGCS2000 */
#define E_CGS2K  (F_CGS2K * (2 - F_CGS2K)) /* 偏心率平方 */
#define Omega_BDS 7.2921150e-5      /* 地球自转角速度 [rad/s]; BDS */
#define GM_BDS 3.986004418E14      /* 地球引力常数 [m^3/s^2]; BDS */
#define GM_GPS 3.986005E14         /* 地球引力常数 [m^3/s^2]; GPS */
#define OMEGAE_GPS 7.2921151467E-5 /* GPS系统地球自转角速度 [rad/s] */
#define OMEGAE_BDS 7.2921150E-5    /* BDS系统地球自转角速度 [rad/s] */

/* 时间系统参数 */
#define GPST_BDT_SEC  14            /* GPS时与北斗时的差值 [s] */
#define GPST_BDT_WEEK  1356         /* GPS时与北斗时的差值 [周] */

/* 卫星系统参数 */
#define MAXCHANNUM 63               /* 最大接收机通道数 */
#define MAXGPSNUM  32               /* 最大GPS卫星数 */
#define MAXBDSNUM 63                /* 最大北斗卫星数 */

/* 文件与数据参数 */
#define MAXRAWLEN 40960     /* 自定义的最大原始数据长度 */

/* CRC校验参数 */
#define POLYCRC32   0xEDB88320u     /* CRC32多项式 */

/* 物理常数 */
#define C_Light 299792458.0         /* 光速 [m/s] */
#define PI 3.1415926535898         /* 圆周率 */
#define F -4.442807633E-10         /* 相对论效应常数 */


/* 信号频率与波长参数 */
/* GPS信号 */
#define FG1_GPS  1575.42E6          /* L1信号频率 [Hz] */
#define FG2_GPS  1227.60E6          /* L2信号频率 [Hz] */
#define WL1_GPS  (C_Light/FG1_GPS)  /* L1信号波长 [m] */
#define WL2_GPS  (C_Light/FG2_GPS)  /* L2信号波长 [m] */

/* 北斗信号 */
#define FG1_BDS  1561.098E6         /* B1信号频率 [Hz] */
#define FG3_BDS  1268.520E6         /* B3信号频率 [Hz] */
#define WL1_BDS  (C_Light/FG1_BDS)  /* B1信号波长 [m] */
#define WL3_BDS  (C_Light/FG3_BDS)  /* B3信号波长 [m] */

/* 标准气象参数 */
#define H0_STD       0              /* 基准高度 [m] */
#define T0_STD       (15 + 273.16)  /* 基准温度 [K] */
#define P0_STD       1013.25        /* 基准气压 [mbar] */
#define RH0_STD      0.5            /* 基准相对湿度 */
#define HW_STD       11000          /* 湿分量尺度高度 [m] */
#define HD_FACTOR1   40136          /* 干分量尺度高度系数1 */
#define HD_FACTOR2   148.72         /* 干分量尺度高度系数2 */