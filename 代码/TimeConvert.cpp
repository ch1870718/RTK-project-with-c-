#include "TimeConvert.h"
void CommonTimetoMjdTime(const COMMONTIME& common, MJDTIME& mjd)   //将公历时间转换为简化儒略日
{
	short year = common.Year;
	unsigned short month = common.Month;
	if (common.Month <= 2)
	{
		year--;
		month += 12;
	}
	double UT = common.Hour + common.Minute / 60.0 + common.Second / 3600.0;
	double JD = int(365.25 * year) + int(30.6001 * (month + 1)) + common.Day + UT / 24.0 + 1720981.5;
	mjd.Days = int(JD - 2400000.5);
	mjd.FracDay = UT / 24.0;
}

void MjdTimeToCommonTime(const MJDTIME& mjd, COMMONTIME& common)   //将简化儒略日转换为公历时间
{
	double JD = mjd.Days + 2400000.5 + mjd.FracDay;
	int a = int(JD + 0.5);
	int b = a + 1537;
	int c = int((b - 122.1) / 365.25);
	int d = int(365.25 * c);
	int e = int((b - d) / 30.6001);
	common.Day = b - d - int(30.6001 * e);
	common.Month = e - 1 - 12 * int(e / 14);
	common.Year = c - 4715 - int((7 + common.Month) / 10);
	common.Hour = int(mjd.FracDay * 24);
	common.Minute = int((mjd.FracDay * 24 - common.Hour) * 60);
	common.Second = (mjd.FracDay * 24 - common.Hour) * 3600.0 - common.Minute * 60.0;
}

void MjdTimeToGPSTime(const MJDTIME& mjd, GPSTIME& gps)   //将简化儒略日转换为GPS时间
{
	gps.Week = int((mjd.Days + mjd.FracDay - 44244) / 7);
	gps.SecOfWeek = (mjd.Days + mjd.FracDay - 44244 - gps.Week * 7) * 86400;
}

void GPSTimeToMjdTime(const GPSTIME& gps, MJDTIME& mjd)   //将GPS时间转换为简化儒略日
{
	mjd.Days = 44244 + gps.Week * 7 + int(gps.SecOfWeek / 86400);
	mjd.FracDay = (gps.SecOfWeek - int(gps.SecOfWeek / 86400) * 86400) / 86400.0;
}

void CommonTimetoGPSTime(const COMMONTIME& common, GPSTIME& gps)   //将公历时间转换为GPS时间
{
	MJDTIME mjd;
	CommonTimetoMjdTime(common, mjd);
	MjdTimeToGPSTime(mjd, gps);
}

void GPSTimeToCommonTime(const GPSTIME& gps, COMMONTIME& common)   //将GPS时间转换为公历时间
{
	MJDTIME mjd;
	GPSTimeToMjdTime(gps, mjd);
	MjdTimeToCommonTime(mjd, common);
}

