#include"Decode.h"
//字节流到特定数据类型的转换
unsigned int UI4(unsigned char* p)    //无符号整型unsigned int
{
	unsigned int u;
	memcpy(&u, p, 4);
	return u;
}
int I4(unsigned char* p)    //有符号整型int
{
	int u;
	memcpy(&u, p, 4);
	return u;
}
double D8(unsigned char* p)    //双精度浮点型double
{
	double u;
	memcpy(&u, p, 8);
	return u;
}
short I2(unsigned char* p)    //短整型short
{
	short u;
	memcpy(&u, p, 2);
	return u;
}
float F4(unsigned char* p)   //单精度浮点型float
{
	float u;
	memcpy(&u, p, 4);
	return u;
}
unsigned short UI2(unsigned char* p)     //无符号短整型unsigned short
{
	unsigned short u;
	memcpy(&u, p, 2);
	return u;
}
unsigned int Crc32(unsigned char* buff, int len)//解释：^是异或、&是提取低位、>>是右移
{
	unsigned int crc = 0;
	for (int i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (int j = 0; j < 8; j++)
		{
			if (crc & 1)
				crc = (crc >> 1) ^ POLYCRC32;
			else
				crc >>= 1;
		}
	}
	return crc;
}
//各种导航数据类型的解码
void DecodeRange(unsigned char* data, int len, EPOCHOBSDATA* obs) 
{
	EPOCHOBSDATA Curobs;
	unsigned char* p = data + 28;
  //1.总的观测时间
	Curobs.Time.Week = UI2(data + 14);
	Curobs.Time.SecOfWeek = UI4(data + 16) * 1E-3;
  //2.总的观测值数量
    int ObsNum = UI4(p);
  //3.各卫星观测值
	//3.1初始化历元观测值容器
	//memset(Curobs.SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));
	//3.2遍历
	p += 4;int k = 0;
	for (int i = 0 ; i < ObsNum; i++, p += 44)
	{	
		//3.2.1各卫星关键中间量
		unsigned int ChanStatus = UI4(p + 40);
		//3.2.2筛选特定系统类型、信号类型的卫星
		int SatSystem = (ChanStatus >> 16) & 0x07;
		int SigType = (ChanStatus >> 21) & 0x1F;
		GNSSsys sys;int Freq;double wl;
		switch (SatSystem)
		{
		case 0: {sys = GPS;
				switch (SigType) { case 0: Freq = 0; wl = WL1_GPS; break; case 9: Freq = 1; wl = WL2_GPS; break; default: continue; }break;}
		case 4: {sys = BDS;
				switch (SigType) { case 0: case 4: Freq = 0; wl = WL1_BDS; break; case 2: case 6: Freq = 1; wl = WL3_BDS; break; default: continue; }break;}
		default: continue;
		}
		//3.2.3卫星号对齐地填充容器中卫星观测值
		int Prn = UI2(p);
		int n = 0;
		for (; n < MAXCHANNUM; n++) {
			if (Curobs.SatObs[n].Prn == Prn && Curobs.SatObs[n].System == sys) break;
			if (Curobs.SatObs[n].Prn == 0) { k =(k<n)?n: k; break; }//遇到obs中在原obs结构体中未有的卫星则记录其卫星号，很大可能卫星号扩容
		}
		int CodeLockedFlag = (ChanStatus >> 12) & 0x01;
		int PhaseLockFlag = (ChanStatus >> 10) & 0x01;
		int ParityFlag = (ChanStatus >> 11) & 0x01;
		Curobs.SatObs[n].Prn = Prn;
		Curobs.SatObs[n].System = sys;
		Curobs.SatObs[n].P[Freq] = CodeLockedFlag ? D8(p + 4) : 0.0;
		Curobs.SatObs[n].L[Freq] = -wl * (PhaseLockFlag  ? D8(p + 16) : 0.0);
		Curobs.SatObs[n].D[Freq] = -wl * F4(p + 28);
		Curobs.SatObs[n].cn0[Freq] = F4(p + 32);
		Curobs.SatObs[n].LockTime[Freq] = F4(p + 36);
		Curobs.SatObs[n].half[Freq] = ParityFlag;
	}
	//4.总的卫星数量
	Curobs.SatNum = k + 1;

	for (int i = 0; i < Curobs.SatNum; i++)
	{
		int j = 0;
		for (; j < obs->SatNum; j++)if ((Curobs.SatObs[i].System == obs->SatObs[j].System) && (Curobs.SatObs[i].Prn == obs->SatObs[j].Prn))break;
		if (Curobs.SatObs[i].LockTime[0] < obs->SatObs[j].LockTime[0]|| Curobs.SatObs[i].LockTime[1] < obs->SatObs[j].LockTime[1])Curobs.SatObs[i].Valid = false;
	}
	*obs = Curobs;
}
void DecodeBdsEph(unsigned char* data, int len, GPSEPHREC beph[])
{
	unsigned char* p = data + 28;
	//检查卫星号
	int prn = UI4(p);
	if (prn < 1 || prn >= MAXBDSNUM) return;
	//填充
	GPSEPHREC* eph;
	eph = beph + prn - 1;

	eph->PRN = prn;
	eph->System = BDS;
	eph->TOC.Week = eph->TOE.Week = UI4(p + 4);
	eph->SVHealth = UI4(p + 16);    //卫星健康状态
	eph->TGD1 = D8(p + 20);
	eph->TGD2 = D8(p + 28);
	eph->IODC = UI4(p + 36);
	eph->TOC.SecOfWeek = UI4(p + 40);
	eph->ClkBias = D8(p + 44);
	eph->ClkDrift = D8(p + 52);
	eph->ClkDriftRate = D8(p + 60);
	eph->IODE = UI4(p + 68);
	eph->TOE.SecOfWeek = UI4(p + 72);
	eph->SqrtA = D8(p + 76);
	eph->e = D8(p + 84);
	eph->omega = D8(p + 92);
	eph->DeltaN = D8(p + 100);
	eph->M0 = D8(p + 108);
	eph->OMEGA = D8(p + 116);
	eph->OMEGADot = D8(p + 124);
	eph->i0 = D8(p + 132);
	eph->iDot = D8(p + 140);
	eph->Cuc = D8(p + 148);
	eph->Cus = D8(p + 156);
	eph->Crc = D8(p + 164);
	eph->Crs = D8(p + 172);
	eph->Cic = D8(p + 180);
	eph->Cis = D8(p + 188);
}
void DecodeGpsEph(unsigned char* data, int len, GPSEPHREC geph[])
{
	unsigned char* p = data + 28;
	//检查卫星号
	int prn = UI4(p);
	if (prn < 1 || prn >= MAXGPSNUM) return;
	//填充
	GPSEPHREC* eph;
	eph = geph + prn - 1;
	eph->PRN = prn;
	eph->System = GPS;
	eph->SVHealth = UI4(p + 12);    //卫星健康状态
	eph->TOC.Week = eph->TOE.Week = UI4(p + 24);
	eph->TOE.SecOfWeek = D8(p + 32);
	eph->SqrtA = sqrt(D8(p + 40));
	eph->DeltaN = D8(p + 48);
	eph->M0 = D8(p + 56);
	eph->e = D8(p + 64);
	eph->omega = D8(p + 72);
	eph->Cuc = D8(p + 80);
	eph->Cus = D8(p + 88);
	eph->Crc = D8(p + 96);
	eph->Crs = D8(p + 104);
	eph->Cic = D8(p + 112);
	eph->Cis = D8(p + 120);
	eph->i0 = D8(p + 128);
	eph->iDot = D8(p + 136);
	eph->OMEGA = D8(p + 144);
	eph->OMEGADot = D8(p + 152);
	eph->IODC = UI4(p + 160);
	eph->TOC.SecOfWeek = D8(p + 164);
	eph->TGD1 = D8(p + 172);
	eph->ClkBias = D8(p + 180);
	eph->ClkDrift = D8(p + 188);
	eph->ClkDriftRate = D8(p + 196);
}
void DecodePsrPos(unsigned char* data, int len, POSRES* pos)
{
	unsigned char* p = data + 28;
	// 大地坐标
	GEOCOOR geo;
	geo.latitude = D8(p + 8) * PI / 180;
	geo.longitude = D8(p + 16) * PI / 180;
	geo.height = D8(p + 24) + F4(p + 32);
	// 大地坐标转直角坐标而后填充
	XYZ xyz;
	BLHToXYZ(geo, xyz, R_WGS84, E_WGS84);
	pos->Pos[0] = xyz.x;pos->Pos[1] = xyz.y;pos->Pos[2] = xyz.z;
}
//判断数据类型并解码

bool DecodeNovOem7Dat(vector<unsigned char> &buff_all,int &len_used, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos) //解码
{
	/*
	//1.打开文件
	ifstream file(filename, ios::binary);
	if (!file) { cerr << "无法打开Msg对应文件" << endl; return; }
	//2.文件内容全部转移到buff_all
	file.seekg(0, ios::end);
	int fileSize = file.tellg();
	file.seekg(0, ios::beg);
	vector<unsigned char> buff_all(fileSize);
	file.read(reinterpret_cast<char*>(buff_all.data()), fileSize);
	*/
	//循环过程的前置准备
	auto currentPos = buff_all.begin();//初始化搜寻中的指针位置
	vector<unsigned char> pattern = { 0xAA, 0x44, 0x12 };//定义搜寻的内容
	//3.循环过程：一次可能有效的消息的获取、有效性判断和而后的有效消息的id提取和有效消息的信息提取
	while (1)
	{
		//3.1获取一条可能有效的消息到buff_once
		  //在一个buff_all中firstPos之后的内容必定是一条buff_once的前部分，如果有secondPos则firstPos与secondPos之间作为一个buff_once，如果没有secondPos则将firstPos之后的内容作为下一个buff_all的前部分
		auto firstPos = search(currentPos, buff_all.end(), pattern.begin(), pattern.end());
		if (firstPos == buff_all.end()) { len_used= firstPos-buff_all.begin(); break; }
		auto secondPos = search(firstPos + pattern.size(), buff_all.end(), pattern.begin(), pattern.end());
		if (secondPos == buff_all.end()) { len_used= secondPos - buff_all.begin(); break; }
		vector<unsigned char> buff_once = {};
		buff_once.insert(buff_once.begin(), firstPos, secondPos);
		//3.2buff_once的有效性判断
			//3.2.1字节长度条件的判断
		if (buff_once.size() < 28) { currentPos = secondPos; continue; }
		unsigned int msglen = UI2(buff_once.data() + 8);
		if (buff_once.size() < 28 + msglen + 4) { currentPos = secondPos; continue; }
			//3.2.2 crc条件的判断
		unsigned int crc = UI4(buff_once.data() + 28 + msglen);
		if (Crc32(buff_once.data(), 28 + msglen) != crc) { currentPos = secondPos; continue; }
		//3.3到此为止，说明这条消息有效，提取消息的id
		unsigned int id = UI2(buff_once.data() + 4);
		//3.4根据id对这条消息进行解码
				//eph本身具有很长的时效性，pos时效性不能定论，obs很短时效性 
				
				//文件的内容是时间上连续的且各种类型消息分布零散，则
				//eph和pos的出现前不会有太多浪费的obs且出现后也有相对长的时效性，遇到obs可以直接从该buff_all的解算跳出来计算预估的pos，buff_all的剩余部分给下一条buff_all
				//流的一条内容是一个时间段的各种类型消息有规律的分布，则
				//不需要接续和为了接续而跳出，最稳妥的做法是读完整条内容
		bool have_obs = false;
		switch (id)
		{
		case 43:
			//memset(obs->SatObs, 0, sizeof(obs->SatObs));
			DecodeRange(buff_once.data(), msglen, obs);
			have_obs = true;
			break;
		case 7:
			DecodeGpsEph(buff_once.data(), msglen, geph);
			break;
		case 1696:
			DecodeBdsEph(buff_once.data(), msglen, beph);
			break;
		case 42:
			DecodePsrPos(buff_once.data(), msglen, pos);
			break;
		default:
			//printf("未知ID: 0x%04X\n", id);
			break;
		}
		currentPos = secondPos;
		if (have_obs) { len_used = currentPos - buff_all.begin(); rotate(buff_all.begin(), currentPos, buff_all.end()); return true; }
	}
	return false;
}

