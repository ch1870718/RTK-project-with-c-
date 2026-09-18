#include"allnecehead.h"
#include "Error.h"
#include "sockets.h"
#include "SPP.h"
#include "Decode.h"
#include"SDAStation.h"
#include"lambda.h"
#include"KFC.h"
#include<iostream>
#include<fstream>
#include<iomanip>
using namespace std;

//readmode mode=ip;
//int main11()
//{
//	EPOCHOBSDATA Obs;    //一个历元的观测值数据结构体
//	GPSEPHREC Geph[MAXGPSNUM], Beph[MAXBDSNUM];    //一个历元星历数据结构体
//	POSRES Res, readRes;//观测值计算的接收机信息，文件读取的接收机信息
//	ENUCOOR enu;XYZ Pos, readPos;
//	vector<unsigned char> buff_all(MAXRAWLEN); int len_used = MAXRAWLEN;
//	ofstream makefile("测试.txt", ios::app);
//	cout << "请输入解码模式:（1）文件模式（2）实时数据流；" << endl;
//	int temp;cin >> temp;
//	mode = static_cast<readmode>(temp);
//	if (mode == 1)
//	{
//		string filename = "C:\\Users\\28638\\Downloads\\oem719-202504011900-1.bin";
//		ifstream file(filename, ios::binary);
//		if (!file) { cerr << "无法打开对应文件" << endl; return 0; }
//		int endpos = (file.seekg(0, ios::end), file.tellg()); int remainbytenum = endpos;
//		file.seekg(0, ios::beg);
//		while (remainbytenum >= MAXRAWLEN)
//		{
//			file.read(reinterpret_cast<char*>(buff_all.data())+ MAXRAWLEN-len_used, len_used);
//			remainbytenum = endpos - file.tellg();
//			DecodeNovOem7Dat(buff_all, len_used, &Obs, Geph, Beph, &readRes,mode);
//			DetectOutlier(&Obs);
//			if (SPP(&Obs, Geph, Beph, &Res))SPV(&Obs, &Res);
//
//			Pos.x = Res.Pos[0], Pos.y = Res.Pos[1], Pos.z = Res.Pos[2];
//			readPos.x = readRes.Pos[0], readPos.y = readRes.Pos[1], readPos.z = readRes.Pos[2];
//			ToENU(readPos, Pos, enu, R_WGS84, E_WGS84);
//			double distance = sqrt(pow(readPos.x - Pos.x, 2) + pow(readPos.y - Pos.y, 2) + pow(readPos.z - Pos.z, 2));
//			cout << fixed << setprecision(3)
//				<< "X:" << Pos.x << " Y:" << Pos.y << " Z:" << Pos.z << " GPS_Clk" << Res.GPS_clockBias << " BDS_Clk" << Res.BDS_clockBias << " PDOP:" << Res.PDOP << " σ:" << Res.SigmaPos
//				<< " E:" << enu.east << " N:" << enu.north << " U:" << enu.up << " distance:" << distance << endl;
//
//		}
//	}
//	else if(mode == 2) 
//	{
//		SOCKET NetGps;
//		if (OpenSocket(NetGps, "47.114.134.129", 7190) == false) { cout << "连接ip失败" << endl; }
//		//if (OpenSocket(NetGps, "8.148.22.229", 5002) == false) { cout << "连接ip失败" << endl; }
//		makefile <<left<< fixed << setprecision(3)
//			<< setw(30) << "X" << setw(30) << " Y" << setw(30) << " Z" << setw(30) << " E" << setw(30) << " N" << setw(30) << " U" << endl;
//		int k = 0;
//		while (true)
//		{
//			Sleep(980);
//			if (recv(NetGps, reinterpret_cast<char*>(buff_all.data()), MAXRAWLEN, 0) > 0)
//			{
//				DecodeNovOem7Dat(buff_all, len_used, &Obs, Geph, Beph, &readRes,mode);
//				DetectOutlier(&Obs);
//				if (SPP(&Obs, Geph, Beph, &Res))SPV(&Obs, &Res);
//
//				Pos.x = Res.Pos[0], Pos.y = Res.Pos[1], Pos.z = Res.Pos[2];
//				readPos.x = readRes.Pos[0], readPos.y = readRes.Pos[1], readPos.z = readRes.Pos[2];
//				ToENU(readPos, Pos, enu, R_WGS84, E_WGS84);
//				double distance = sqrt(pow(readPos.x - Pos.x, 2) + pow(readPos.y - Pos.y, 2) + pow(readPos.z - Pos.z, 2));
//				if (fabs(distance) < 1e-4 || fabs(distance) >= 10)continue;
//				k++; if (k >= 30000)break;
//				makefile <<left<< fixed << setprecision(3)
//					<< setw(30) <<  Pos.x << setw(30)<< Pos.y << setw(30) << Pos.z << setw(30) << enu.east << setw(30) << enu.north << setw(30) << enu.up << endl;
//				cout << fixed << setprecision(3)
//					<< "X:" << Pos.x << " Y:" << Pos.y << " Z:" << Pos.z << " GPS_Clk" << Res.GPS_clockBias << " BDS_Clk" << Res.BDS_clockBias << " PDOP:" << Res.PDOP << " σ:" << Res.SigmaPos
//					<< " E:" << enu.east << " N:" << enu.north << " U:" << enu.up << " distance:" << distance << endl;
//			}
//		} 
//
//	}
//	else {
//		cout << "输入错误，请重新运行程序！" << endl;
//		return -1;
//	}
//
//	return 0;
//
//
//
//
//}

int main8()
{
	FILE* FObs_rover;
	FILE* FObs_base;
	//errno_t err_rover = fopen_s(&FObs_rover, "C:\\Users\\28638\\Desktop\\6.21weixing\\oem719-202202021500-rover.bin", "rb");
	//errno_t err_base = fopen_s(&FObs_base, "C:\\Users\\28638\\Desktop\\6.21weixing\\oem719-202202021500-base.bin", "rb");

	errno_t err_rover = fopen_s(&FObs_rover, "C:\\Users\\28638\\Desktop\\FH\\oem719-202510311730-rover.bin", "rb");
	errno_t err_base = fopen_s(&FObs_base, "C:\\Users\\28638\\Desktop\\FH\\oem719-202510311730-base.bin", "rb");
	
	SOCKET NetGps1, NetGps2;
	char ip1[] ="8.148.22.229" , ip2[] = "47.114.134.129";
	unsigned short port1 = 7002, port2 = 7190;//流动站和基站
	if (OpenSocket(NetGps2, ip2, port2) == false) { cout << "流动站连接ip失败" << endl; }
	if (OpenSocket(NetGps1, ip1, port1) == false) { cout << "基准站连接ip失败" << endl; }
	//if (OpenSocket(NetGps2, ip1, port1) == false) { cout << "流动站连接ip失败" << endl; }
	//if (OpenSocket(NetGps1, ip1, 5002) == false) { cout << "基准站连接ip失败" << endl; }
	RAWDATA rawdata_pre; 
	

	ofstream ofs("C:\\Users\\28638\\Desktop\\FH\\ceshi.txt");
	ofs << left
		<< setw(8) << "GPSNum" << setw(8) << "BDSNum"
		<< setw(10) <<"ratio"
		<< setw(20)<< "x" << setw(20) << "y" << setw(20) <<"z" << setw(20) << "PDOP"<<  endl;
	while (!feof(FObs_rover))
	{
		static RAWDATA rawdata;
		ALLSDOBS allsdaobs;
		ALLDDOBS allddobs;
		//Sleep(980);
		//if (!TimeSyn_ip(&NetGps1, &NetGps2, &rawdata))continue;
		if (!TimeSyn_data(FObs_rover, FObs_base, &rawdata))continue;
		Matrix amb, Q_amb;
		Matrix F_, s; int m = 2;
		DetectOutlier(&rawdata.rover.Obs);
		DetectOutlier(&rawdata.base.Obs);
		if (!SPP(&rawdata.rover.Obs, rawdata.rover.GpsEph, rawdata.rover.BdsEph, &rawdata.rover.ObsPOS))continue;
		if (!SPP(&rawdata.base.Obs, rawdata.base.GpsEph, rawdata.base.BdsEph, &rawdata.base.ObsPOS))continue;
		SD(&rawdata, &allsdaobs);
		Detect(&allsdaobs);
		SelectforDD(&rawdata, &allsdaobs, &allddobs);
		if (!RTK_LSQ(&rawdata,&allsdaobs,&allddobs, &amb, &Q_amb))continue;
		lambda(2, &amb, &Q_amb, &F_, &s);
		if(s(1, 0) / s(0, 0) >= 3)RTK_lambdaed(&rawdata, &allsdaobs, &allddobs, &amb, &Q_amb, 2, &F_, &s);
		int nnn = (s(1, 0) / s(0, 0) >= 3) ? 1 : 2;

		double dis_betw[3] = {};
		for (int m = 0; m < 3; m++)dis_betw[m] = rawdata.rover.ObsPOS.Pos[m] - rawdata.base.ReadPOS.Pos[m];
		double dx =sqrt( pow(rawdata.rover.ObsPOS.Pos[0] - rawdata.base.ReadPOS.Pos[0], 2) + pow(rawdata.rover.ObsPOS.Pos[1] - rawdata.base.ReadPOS.Pos[1], 2) + pow(rawdata.rover.ObsPOS.Pos[2] - rawdata.base.ReadPOS.Pos[2], 2));
		COMMONTIME commontime;
		GPSTimeToCommonTime(rawdata.rover.Obs.Time, commontime);
		XYZ xyz; GEOCOOR geo;
		xyz.x = rawdata.rover.ObsPOS.Pos[0];
		xyz.y = rawdata.rover.ObsPOS.Pos[1];
		xyz.z = rawdata.rover.ObsPOS.Pos[2];
		XYZToBLH(xyz, geo, R_WGS84, E_WGS84);
		cout << rawdata.rover.Obs.Time.SecOfWeek << " " << rawdata.base.Obs.Time.SecOfWeek << endl;
		//cout << rawdata.rover.Obs.Time.Week << " " << rawdata.base.Obs.Time.SecOfWeek << endl;
		//cout << commontime.Year << "-" << commontime.Month << "-" << commontime.Day << " " << 8+commontime.Hour << ":" << commontime.Minute << ":" << int(commontime.Second) << endl;
		//cout<< "双差数: " << allddobs.Num ;
		//cout << "   Ratio: " << s(1, 0) / s(0, 0)<<endl;
		cout  << fixed << setprecision(8) << geo.latitude * 180 / PI << " ," << geo.longitude * 180 / PI << ", " << geo.height << endl;
		//cout << " 基线向量：( " << dis_betw[0] << " , " << dis_betw[1] << " , " << dis_betw[2] <<" )" <<endl;
	    
		ofs<<left
			<< fixed << setprecision(0) << rawdata.rover.Obs.Time.Week << " " << rawdata.base.Obs.Time.SecOfWeek<<"  "
			//<< setw(8)<<allddobs.GPSNum+1<< setw(8) << allddobs.BDSNum + 1
			//<< setw(10) << fixed << setprecision(6) << s(1, 0) / s(0, 0)
			<< fixed << setprecision(8) << geo.latitude * 180 / PI << "  " << geo.longitude * 180 / PI << "  " << geo.height <<"  "<<nnn<< endl;
			//<< setw(20) << fixed << setprecision(6) << rawdata.rover.ObsPOS.Pos[0] << setw(20) << rawdata.rover.ObsPOS.Pos[1] << setw(20) << rawdata.rover.ObsPOS.Pos[2] 
			//<< setw(20) << fixed << setprecision(6) << rawdata.rover.ObsPOS.PDOP<<endl;
		/*ofs2 << left
			<< setw(8) << fixed << setprecision(0)<< rawdata.rover.Obs.Time.Week << setw(8) << rawdata.rover.Obs.Time.SecOfWeek;
			ofs2 << setw(20) << fixed << setprecision(10) << geo.latitude << setw(20) << geo.longitude << setw(20) << geo.height << endl;*/
	}
	

	return 0;
}
int main()
{
	FILE* FObs_rover;
	FILE* FObs_base;
	//errno_t err_rover = fopen_s(&FObs_rover, "C:\\Users\\28638\\Desktop\\6.21weixing\\oem719-202202021500-rover.bin", "rb");
	//errno_t err_base = fopen_s(&FObs_base, "C:\\Users\\28638\\Desktop\\6.21weixing\\oem719-202202021500-base.bin", "rb");
	errno_t err_rover = fopen_s(&FObs_rover, "C:\\Users\\28638\\Desktop\\FH\\oem719-202510311730-rover.bin", "rb");
	errno_t err_base = fopen_s(&FObs_base, "C:\\Users\\28638\\Desktop\\FH\\oem719-202510311730-base.bin", "rb");
     //errno_t err_rover = fopen_s(&FObs_rover, "C:\\Users\\28638\\Downloads\\short-baseline-data\\oem719-202510271500-rover.bin", "rb");
	//errno_t err_base = fopen_s(&FObs_base, "C:\\Users\\28638\\Downloads\\short-baseline-data\\oem719-202510271500-base.bin", "rb");

	SOCKET NetGps1, NetGps2;
	/*const char ip1[], const unsigned short port1, const char ip2[], const unsigned short port2,*/
	char ip1[] = "8.148.22.229", ip2[] = "47.114.134.129";
	unsigned short port1 = 7002, port2 = 7190;//流动站和基站
	if (OpenSocket(NetGps2, ip2, port2) == false) { cout << "流动站连接ip失败" << endl; }
	if (OpenSocket(NetGps1, ip1, port1) == false) { cout << "基准站连接ip失败" << endl; }
	//if (OpenSocket(NetGps2, ip1, port1) == false) { cout << "流动站连接ip失败" << endl; }
	//if (OpenSocket(NetGps1, ip1, 5002) == false) { cout << "基准站连接ip失败" << endl; }
	
	
	ofstream ofs1("C:\\Users\\28638\\Desktop\\FH\\111.txt");
	ofstream ofs2("C:\\Users\\28638\\Desktop\\FH\\222.txt");


	while (!feof(FObs_rover))
	{
		static RAWDATA rawdata;
		ALLSDOBS allsdaobs;
		static ALLDDOBS allddobs, allddobs_before;
		static POSRES pos_before;
		// Sleep(980);
		//if (!TimeSyn_ip(&NetGps1, &NetGps2, &rawdata))continue;
		//cout << "Timeright" << endl;
		if (!TimeSyn_data(FObs_rover, FObs_base, &rawdata))continue;
		
		
		DetectOutlier(&rawdata.rover.Obs);
		DetectOutlier(&rawdata.base.Obs);
		if (!SPP(&rawdata.rover.Obs, rawdata.rover.GpsEph, rawdata.rover.BdsEph, &rawdata.rover.KfcPOS))continue;
		if (!SPP(&rawdata.base.Obs, rawdata.base.GpsEph, rawdata.base.BdsEph, &rawdata.base.KfcPOS))continue;
		SD(&rawdata, &allsdaobs);
		Detect(&allsdaobs);
		SelectforDD(&rawdata, &allsdaobs, &allddobs);


		static RAWDATA rawdata1;
		rawdata1 = rawdata;
		Matrix amb1, Q_amb1;
		Matrix F_1, s1;
		if (!RTK_LSQ(&rawdata1, &allsdaobs, &allddobs, &amb1, &Q_amb1))continue;
		lambda(2, &amb1, &Q_amb1, &F_1, &s1);
		if (s1(1, 0) / s1(0, 0) >= 3)RTK_lambdaed(&rawdata1, &allsdaobs, &allddobs, &amb1, &Q_amb1, 2, &F_1, &s1);
		int nnn = (s1(1, 0) / s1(0, 0) >= 3) ? 1 : 2;
		static RAWDATA rawdata2;
		rawdata2 = rawdata;
		Matrix amb2, Q_amb2;
		Matrix F_2, s2;
		if (!KFC(&allddobs_before, &pos_before, &rawdata2, &allsdaobs, &allddobs, &amb2, &Q_amb2))continue;
		lambda(2, &amb2, &Q_amb2, &F_2, &s2);
		pos_before = rawdata2.rover.KfcPOS;
		allddobs_before = allddobs;
		
		XYZ xyz1; GEOCOOR geo1;
		xyz1.x = rawdata1.rover.ObsPOS.Pos[0];
		xyz1.y = rawdata1.rover.ObsPOS.Pos[1];
		xyz1.z = rawdata1.rover.ObsPOS.Pos[2];
		XYZToBLH(xyz1, geo1, R_WGS84, E_WGS84);

		XYZ xyz2; GEOCOOR geo2;
		xyz2.x = rawdata2.rover.KfcPOS.Pos[0];
		xyz2.y = rawdata2.rover.KfcPOS.Pos[1];
		xyz2.z = rawdata2.rover.KfcPOS.Pos[2];
		XYZToBLH(xyz2, geo2, R_WGS84, E_WGS84);
		
		cout << rawdata1.base.Obs.Time.SecOfWeek << endl;
		ofs1 << left
			<< fixed << setprecision(0) << rawdata1.rover.Obs.Time.Week << " " << rawdata1.base.Obs.Time.SecOfWeek << "  "
			//<< setw(8)<<allddobs.GPSNum+1<< setw(8) << allddobs.BDSNum + 1
			//<< setw(10) << fixed << setprecision(6) << s(1, 0) / s(0, 0)
			<< fixed << setprecision(8) << geo1.latitude * 180 / PI << "  " << geo1.longitude * 180 / PI << "  " << geo1.height << "  " << nnn << endl;
		if (fabs(geo2.height - 41.400044) > 0.5)continue;
		ofs2 << left
			<< fixed << setprecision(0) << rawdata2.rover.Obs.Time.Week << " " << rawdata2.base.Obs.Time.SecOfWeek << "  "
			//<< setw(8)<<allddobs.GPSNum+1<< setw(8) << allddobs.BDSNum + 1
			//<< setw(10) << fixed << setprecision(6) << s(1, 0) / s(0, 0)
			<< fixed << setprecision(8) << geo2.latitude * 180 / PI << "  " << geo2.longitude * 180 / PI << "  " << geo2.height << "  " << 1 << endl;
	
	}




	return 0;
}