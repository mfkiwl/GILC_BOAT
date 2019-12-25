#ifdef WIN32
#include "stdafx.h"
#include <string.h>
#endif
#include "GILC_Config.h"
#include "GILC_IOFile.h"
#include "GILC_Boat_lib.h"
#include "GILC_Vehicle_inner.h"

#include "mem.h"
#ifdef STM32
#include "malloc_allocator.h"
#else
#define malloc_allocator allocator
#endif
int gnss_static = 0;
int gilc_static = 0;
class GilcProcess
{
public:
	gilc_cfg_t s_cfgdata;
	gilc_raw_t stRaw;
	gilc_raw_t stRaw_tmp;
	gilc_result_inner_t stRstInner;
	ekf_X_t stEkfX;
	
	double Gpospre[3];
	GIProcess gipro;
	KinAlign kinalign;
	Filter iir_filter;
	CLCData ilcd,pre_ilcd;
	DebugFile debugfile;
	int raw_cnt;
	int dbg_cnt;
	int rst_cnt;
	bool GetGPS;
	bool bAlign;
	double eb[3],db[3];
	char cGilcInitMsg[2048];

	double dInstallMat[9];
	double dInstallAttCfg[3];
	
	double dGnss2OutPointLever[3];
	vector<double> Rst_vel;
	vector<double> Rst_heading;
	vector<double> Rst_lat;
	vector<double> Rst_lon;
	vector<double> Rst_alt;
	vector<double> Raw_gnss_speed;
	vector<double> Rst_speed;
	double Rst_vn_pre;
	double Rst_ve_pre;
	double Rst_speed_pre;
	int static_num;
	int no_static_num;
	int iGnssSec_pre;
	GilcProcess(void){};
	
	int GILC_Init(gilc_cfg_t* cfgdata);
	void GILC_Update_Status(gilc_result_t* pstOut,int iGilcStatus,int iGnssStatus,bool bDualAntAvail);
	void GILC_GnssRaw_Correct(gilc_raw_t *pRaw);
	void GILC_ImuAxis_Correct(gilc_raw_t *pRaw);
	int GILC_Raw2CLCData(gilc_raw_t *pRaw,CLCData *pIlcData);
	int GILC_Rst_Filter(gilc_result_t* pstOut);//�Ե��١�����λ��ȥ������ĵ�
	void GILC_Rst_Vel_smooth(GIProcess* gipro);//�ٶȷ���ƽ��
	gilc_ret_e GILC_LoadRaw_byStrn(char *buff,gilc_raw_t *pRaw);
	gilc_ret_e GILC_PROCESS_Vehicle(gilc_raw_t* pstRaw,gilc_result_t* pstOut);
	gilc_ret_e GILC_PROCESS_Vehicle_byStrn(char* buff,gilc_result_t* pstOut);
/*	
	void printf_speed_plot(gilc_result_t *pstOut);
	void printf_static_plot(gilc_result_t *pstOut);
	void printf_difposb_plot(gilc_result_t *pstOut);
*/
};

#define STATIC_DEBUG 1
int IsStatic_debug(CLCData& ilcd, double Acc[3], double Gyro[3], double stdAcc, double stdGyro)
{
	static double dAcc_amvd_max = 100;
	static double dGyo_ared_max = 100;
	static vector<double, malloc_allocator<double> > axyz_iir[3],gxyz_iir[3];
	int i = 0;
	double dAcc_std[3] = {0};
	double dGyro_rms[3] = {0};
	double dAcc_amvd = {0};
	double dGyro_ared = 0;
	int iTemp = 1;

	if(gxyz_iir[0].size()<100)
	{
		for(i=0;i<3;i++)
		{
			axyz_iir[i].push_back(Acc[i]);
			gxyz_iir[i].push_back(Gyro[i]);
		}
		return 0;
	}
	else
	{
		for(i=0;i<3;i++)
		{
			axyz_iir[i].erase(axyz_iir[i].begin());
			axyz_iir[i].push_back(Acc[i]);
			gxyz_iir[i].erase(gxyz_iir[i].begin());
			gxyz_iir[i].push_back(Gyro[i]);
			dAcc_std[i] = GetAveStd(axyz_iir[i],1);
			dGyro_rms[i]= GetAveStd(gxyz_iir[i],2);
			dAcc_amvd += dAcc_std[i] * dAcc_std[i] / stdAcc;
			dGyro_ared+= dGyro_rms[i] * dGyro_rms[i] / stdGyro;
		}	

#if (STATIC_DEBUG)
		iTemp &= (dAcc_amvd < dAcc_amvd_max ? 1 : 0);
		iTemp &= (dGyro_ared < dGyo_ared_max ? 1 : 0);
		ilcd.acc_iir[0] = dAcc_amvd;
		ilcd.acc_iir[1] = dGyro_ared;
		ilcd.acc_iir[2] = iTemp;
#else
		iTemp &= (dAcc_amvd < dAcc_amvd_max ? 1 : 0);
		iTemp &= (dGyro_ared < dGyo_ared_max ? 1 : 0);
#endif
	}
	return iTemp;
}

void printf_rtkplot(FILE *fd,int week,double imutimetarge,double pos[3],int stat)
{
	if (!fd) return;

	double ep[6] = { 0 };
	gtime_t gt = gpst2time(week, imutimetarge-1); /*rtklib plot use 17s*/
	time2epoch(gt, ep);

	fprintf(fd,"%4d/%02d/%02d %02d:%02d:%06.3f %15.9f%15.9f%11.4f%4d%4d\n",
		int(ep[0]), int(ep[1]), int(ep[2]), int(ep[3]), int(ep[4]), ep[5], pos[0], pos[1], pos[2], stat, 10);
}

/*dsf90:������������ϵ�£�����λ�ò�*/
double difpos_enu(double pospre[3], double poscur[3], double dpos_enu[3])
{
	double rr0[3] = { 0.0 }, rr1[3] = { 0.0 }, drr[3] = { 0.0 };
	pos2ecef(pospre, rr0);
	pos2ecef(poscur, rr1);
	Mminn(rr1, rr0, drr, 3, 1);
	ecef2enu(pospre, drr, dpos_enu);
	return sqrt(dpos_enu[0] * dpos_enu[0] + dpos_enu[1] * dpos_enu[1] + dpos_enu[2] * dpos_enu[2]);
}
#if 0
void GilcProcess::printf_static_plot(gilc_result_t *pstOut)
{
	static FILE* fd_rst_plot = NULL;
	if (!fd_rst_plot)
	{
		char cFileNamePath[128] = {0};
		sprintf(cFileNamePath, "%sstatic.plot", s_cfgdata.debug_outfile_path);
		fd_rst_plot = fopen(cFileNamePath, "wt");
		if (!fd_rst_plot)
			gilc_log("open file err! %s\r\n", cFileNamePath);
	}
	else
	{
		double pos[3] = { pstOut->lla[0],pstOut->lla[1],0};
		int static_flag = 0;
		if(pstOut->car_status == 1)
			static_flag |= 0x01;
		
		if(pstOut->gilc_status!=GILC_RET__RST_INS_MODE && ilcd.gnss_speed < 0.05)
		{
			static_flag |= 0x02;
		}
		pos[2] = static_flag*10;
		printf_rtkplot(fd_rst_plot, ilcd.week, ilcd.imutimetarge, pos, pstOut->gilc_status);
	}		
}

void GilcProcess::printf_speed_plot(gilc_result_t *pstOut)
{
	static FILE* fd_rst_plot = NULL;
	if (!fd_rst_plot)
	{
		char cFileNamePath[128] = {0};
		sprintf(cFileNamePath, "%sspeed.plot", s_cfgdata.debug_outfile_path);
		fd_rst_plot = fopen(cFileNamePath, "wt");
		if (!fd_rst_plot)
			gilc_log("open file err! %s\r\n", cFileNamePath);
	}
	else
	{
		double pos[3] = { pstOut->lla[0],pstOut->lla[1],0};
		pos[2] = pstOut->speed;
		static double dImuTime_pre = 0;
		if(dImuTime_pre<1e-6)
		{
			dImuTime_pre = pstOut->second;
		}
		int static_flag = (int)((pstOut->second - dImuTime_pre)/0.01);
		dImuTime_pre = pstOut->second;
		if(static_flag>1)
			printf_rtkplot(fd_rst_plot, ilcd.week, ilcd.imutimetarge, pos, static_flag);
	}		
}

void GilcProcess::printf_difposb_plot(gilc_result_t *pstOut)
{
	static FILE* fd_rst_plot = NULL;
	if (!fd_rst_plot)
	{
		char cFileNamePath[128] = {0};
		sprintf(cFileNamePath, "%sdifposb.plot", s_cfgdata.debug_outfile_path);
		fd_rst_plot = fopen(cFileNamePath, "wt");
		if (!fd_rst_plot)
			gilc_log("open file err! %s\r\n", cFileNamePath);
	}
	else
	{
		double pos[3] = { stRstInner.dpos_sync[0],stRstInner.dpos_sync[1],stRstInner.dpos_sync[2]};
		printf_rtkplot(fd_rst_plot, ilcd.week, ilcd.gpstimetarge, pos, pstOut->car_status);
	}		
}
#endif
/*add by dsf90, 2018.5.15*/
void GilcProcess::GILC_ImuAxis_Correct(gilc_raw_t *pRaw)
{
	if(s_cfgdata.bRowScaleCfgUse)
	{
		pRaw->memsdate.gyro[0] =  pRaw->memsdate.alldata[s_cfgdata.gyro_row[0]-1] * s_cfgdata.gyro_scale[0];
		pRaw->memsdate.gyro[1] =  pRaw->memsdate.alldata[s_cfgdata.gyro_row[1]-1] * s_cfgdata.gyro_scale[1];
		pRaw->memsdate.gyro[2] =  pRaw->memsdate.alldata[s_cfgdata.gyro_row[2]-1] * s_cfgdata.gyro_scale[2];
		pRaw->memsdate.accel[0] =  pRaw->memsdate.alldata[s_cfgdata.acc_row[0]-1] * s_cfgdata.acc_scale[0];
		pRaw->memsdate.accel[1] =  pRaw->memsdate.alldata[s_cfgdata.acc_row[1]-1] * s_cfgdata.acc_scale[1];
		pRaw->memsdate.accel[2] =  pRaw->memsdate.alldata[s_cfgdata.acc_row[2]-1] * s_cfgdata.acc_scale[2];	
	}
	else
	{
		pRaw->memsdate.gyro[0] =   pRaw->memsdate.alldata[CFG_GYRO_X_ROW-1] * CFG_GYRO_X_SCALE;
		pRaw->memsdate.gyro[1] =   pRaw->memsdate.alldata[CFG_GYRO_Y_ROW-1] * CFG_GYRO_Y_SCALE;
		pRaw->memsdate.gyro[2] =   pRaw->memsdate.alldata[CFG_GYRO_Z_ROW-1] * CFG_GYRO_Z_SCALE;
		pRaw->memsdate.accel[0] =  pRaw->memsdate.alldata[CFG_ACC_X_ROW-1] * CFG_ACC_X_SCALE;
		pRaw->memsdate.accel[1] =  pRaw->memsdate.alldata[CFG_ACC_Y_ROW-1] * CFG_ACC_Y_SCALE;
		pRaw->memsdate.accel[2] =  pRaw->memsdate.alldata[CFG_ACC_Z_ROW-1] * CFG_ACC_Z_SCALE;
	}	

	Mmulnm(dInstallMat, pRaw->memsdate.gyro, 3, 3, 1, pRaw->memsdate.gyro);
	Mmulnm(dInstallMat, pRaw->memsdate.accel, 3, 3, 1, pRaw->memsdate.accel);
}

void GilcProcess::GILC_Update_Status(gilc_result_t* pstOut,int iGilcStatus,int iGnssStatus,bool bDualAntAvail)
{
	int iGilcStatusChc_l = 0,iGilcStatusChc_h = 0;
	
	pstOut->gilc_status = iGilcStatus;
	if (gipro.bStatic)
		pstOut->car_status = 1;
	else if (gipro.bTurn == 1)
		pstOut->car_status = 3; //3.turn right 
	else if (gipro.bTurn == -1)
		pstOut->car_status = 4; //4.turn left
	else
		pstOut->car_status = 0;

	if(bDualAntAvail)
	{
		switch(iGnssStatus)	
		{
			case 0:
				iGilcStatusChc_h = GILC_STATUS_H__RST_NOTHING;
				break;
			case 1:
				iGilcStatusChc_h = GILC_STATUS_H__SINGLE_HEADING;
				break;
			case 2:
			case 9: /*WAAS*/
				iGilcStatusChc_h = GILC_STATUS_H__RTD_HEADING;
				break;
			case 3:
				iGilcStatusChc_h = GILC_STATUS_H__INS;
				break;
			case 4:
				iGilcStatusChc_h = GILC_STATUS_H__RTK_HEADING;
				break;
			case 5:
				iGilcStatusChc_h = GILC_STATUS_H__FLOAT_HEADING;
				break;
			default:
				iGilcStatusChc_h = GILC_STATUS_H__RST_NOTHING;
				break;
		}
	}
	else
	{
		switch(iGnssStatus)	
		{
			case 0:
				iGilcStatusChc_h = GILC_STATUS_H__RST_NOTHING;
				break;
			case 1:
				iGilcStatusChc_h = GILC_STATUS_H__SINGLE;
				break;
			case 2:
			case 9: /*WAAS*/
				iGilcStatusChc_h = GILC_STATUS_H__RTD;
				break;
			case 3:
				iGilcStatusChc_h = GILC_STATUS_H__INS;
				break;
			case 4:
				iGilcStatusChc_h = GILC_STATUS_H__RTK;
				break;
			case 5:
				iGilcStatusChc_h = GILC_STATUS_H__FLOAT;
				break;
			default:
				iGilcStatusChc_h = GILC_STATUS_H__RST_NOTHING;
				break;
		}
	}	

	switch(iGilcStatus)	
	{
		case GILC_RET__RST_NOTHING:
		case GILC_RET__RST_INITING:
			iGilcStatusChc_l = GILC_STATUS_L__GNSS_MODE;
			break;
		case GILC_RET__RST_RESOLVING:
			iGilcStatusChc_l = GILC_STATUS_L__INIT_MODE;
			break;
		case GILC_RET__RST_STABLE:
			iGilcStatusChc_l = GILC_STATUS_L__GILC_MODE;
			break;
		case GILC_RET__RST_INS_MODE:
			iGilcStatusChc_l = GILC_STATUS_L__INS_MODE;
			break;
		default:
			iGilcStatusChc_l = GILC_STATUS_L__GNSS_MODE;
			break;
	}
	
	pstOut->gilc_status_chc = iGilcStatusChc_h + iGilcStatusChc_l;
}


void GilcProcess::GILC_Rst_Vel_smooth(GIProcess* gipro)
{
	double avg_vel[2] = {0};
	//if (fabs(gipro->ins.vnL[0] - Rst_ve_pre) > 0.2)
	//{
	//	avg_vel[0] = (gipro->ins.vnL[0] + Rst_ve_pre) / 2;
	//}
	//else
	//{
	//	avg_vel[0] = gipro->ins.vnL[0];
	//}
	//if (fabs(gipro->ins.vnL[1] - Rst_vn_pre) > 0.2)
	//{
	//	avg_vel[1] = (gipro->ins.vnL[1] + Rst_vn_pre) / 2;
	//}
	//else
	//{
	//	avg_vel[1] = gipro->ins.vnL[1];
	//}
	avg_vel[0] = (gipro->ins.vn[0] + Rst_ve_pre) / 2;
	avg_vel[1] = (gipro->ins.vn[1] + Rst_vn_pre) / 2;
	gipro->ins.vn[0] = avg_vel[0];
	gipro->ins.vn[1] = avg_vel[1];
	Rst_ve_pre = gipro->ins.vn[0];
	Rst_vn_pre = gipro->ins.vn[1];

	//double blh[3] = {0.0}, rr0[3] = { 0.0 }, enu[3] = { 0.0 };
	//for (int i = 0;i < 3;i++)
	//{
	//	blh[i] = pstOut->lla[i];
	//}
	//pos2ecef(blh, rr0);



}


int GilcProcess::GILC_Rst_Filter(gilc_result_t* pstOut)
{
#if 1
	if (Rst_alt.size() < Filter_NUM)
	{
		//Rst_heading.push_back(pstOut->heading);
		//Rst_vel.push_back(pstOut->speed);
		//Rst_lat.push_back(pstOut->lla[0]);
		//Rst_lon.push_back(pstOut->lla[1]);
		Rst_alt.push_back(pstOut->lla[2]);
		return 0;
	}
	else
	{
		Rst_alt.erase(Rst_alt.begin());
		Rst_alt.push_back(pstOut->lla[2]);
		double  data_alt[Filter_NUM] = { 0 };
		for (int i = 0;i < Filter_NUM;i++)
		{
			data_alt[i] = Rst_alt[i];
		}
		pstOut->lla[2] = dataFilter(data_alt, Filter_NUM);
		return 1;
	}
#endif
}




void GilcProcess::GILC_GnssRaw_Correct(gilc_raw_t *pRaw)
{
	/*NOVATEL*/
	static bool bGGAStatUsed = 0;
	if(!bGGAStatUsed && pRaw->gnssdata.stat) //GNSS GGA stat used check
		bGGAStatUsed = 1;
		
	if (!bGGAStatUsed)
	{
		switch (pRaw->gnssdata.pos_type)
		{
			case 16:
			case 20:
				pRaw->gnssdata.stat = 1;
				break;
			case 17:
			case 18:/*SBAS*/
				pRaw->gnssdata.stat = 2;
				break;
			case 19:
				pRaw->gnssdata.stat = 0;
				break;
			case 32:
			case 33:
			case 34:
				pRaw->gnssdata.stat = 5;
				break;
			case 48:
			case 49:
			case 50:
				pRaw->gnssdata.stat = 4;
				break;
			default:
				gilc_log("libgilc --- raw gnss state error: week %d, sec %f, state %d\r\n",
					pRaw->gnssdata.week, pRaw->gnssdata.second,pRaw->gnssdata.pos_type);
				pRaw->gnssdata.stat = 0;
				break;
		}
	}
	else
	{
		switch (pRaw->gnssdata.stat)
		{
			case 0:
			case 1:
			case 2:
			case 4:
			case 5:
			case 9:/*SBAS*/
				break;
			default:
				gilc_log("libgilc --- raw gnss state error: week %d, sec %f, state %d\r\n",
					pRaw->gnssdata.week, pRaw->gnssdata.second,pRaw->gnssdata.stat);
				pRaw->gnssdata.stat = 0;
				break;
		}
	}

	if(s_cfgdata.eGnssVelMode == GILC_GNSSVEL_MODE__ECEF)
	{
		double vel_ecef[3]={0.0},pos[3]={0.0};
		double vel_enu[3]={0.0};
		vel_ecef[0]=pRaw->gnssdata.vx_ecef;
		vel_ecef[1]=pRaw->gnssdata.vy_ecef;
		vel_ecef[2]=pRaw->gnssdata.vz_ecef;
		pos[0] = pRaw->gnssdata.lat*D2R;
		pos[1] = pRaw->gnssdata.lon*D2R;
		pos[2] = pRaw->gnssdata.alt;		
		//ecef2enu(pos,vel_ecef,vel_enu);
		vel_enu[0] = vel_ecef[0];
		vel_enu[1] = vel_ecef[1];
		vel_enu[2] = vel_ecef[2];
		pRaw->gnssdata.speed   = sqrt(pow(vel_enu[0],2)+pow(vel_enu[1],2));
		pRaw->gnssdata.heading = atan2(vel_enu[0],vel_enu[1])*R2D;
		
		if (pRaw->gnssdata.heading <0) 
			pRaw->gnssdata.heading += 360;
	}

}

int  GilcProcess::GILC_Raw2CLCData(gilc_raw_t *pRaw,CLCData *pIlcData)
{
	if(!(pRaw->bMEMSavail || pRaw->bGPSavail))
	{
		gilc_log("Raw Data error!!! %d,%d\r\n",pRaw->bMEMSavail,pRaw->bGPSavail);
		return GILC_RET__ERR_RAW_NO_UPDATE;
	}

	pIlcData->bMEMSavail=false;
	pIlcData->bGPSavail=false;
	pIlcData->bPPSavail=false;
	/*******************MEMS*********************/
	if(pRaw->bMEMSavail)
	{
		static double dSec_pre = 0;
		pIlcData->imutimetarge = pRaw->imutimetarget;
		if (pIlcData->imutimetarge - dSec_pre > 0.1)
		{
			gilc_log("libgilc --- raw imu time error: week %d, sec %f, last %f \r\n",
				pIlcData->week, pIlcData->imutimetarge,dSec_pre);
			if (pIlcData->imutimetarge - dSec_pre > 1.0 && gipro.bInstallOk)
			{
				GILC_Init(&s_cfgdata);
			}
			dSec_pre = pIlcData->imutimetarge;
			return GILC_RET__ERR_RAW_IMU_TIME;
		}
		dSec_pre = pIlcData->imutimetarge;

		pIlcData->gyo[0]=pRaw->memsdate.gyro[0] * D2R;
		pIlcData->gyo[1]=pRaw->memsdate.gyro[1] * D2R;
		pIlcData->gyo[2]=pRaw->memsdate.gyro[2] * D2R;
		pIlcData->acc[0]=pRaw->memsdate.accel[0] * glv_g;
		pIlcData->acc[1]=pRaw->memsdate.accel[1] * glv_g;
		pIlcData->acc[2]=pRaw->memsdate.accel[2] * glv_g;
		pIlcData->temper = pRaw->memsdate.temper;

		/*dsf90:IMU���ݼ���*/
		for (int i = 0; i < 3; i++)
		{
			if (fabs(pIlcData->acc[i]) >= 10 * glv_g)/*���ٶ����� 10g*/
			{
				gilc_log("libgilc --- raw imu accel error: week %d, sec %f, accel %f %f %f g\r\n", 
					pIlcData->week,pIlcData->imutimetarge,
					pIlcData->acc[0]/glv_g,pIlcData->acc[1]/glv_g,pIlcData->acc[2]/glv_g);
				return GILC_RET__ERR_RAW_IMU_ACCEL_PARAM;
			}
			if (fabs(pIlcData->gyo[i]) >= 1000 * D2R)/*���������� 1000deg/s*/
			{
				gilc_log("libgilc --- raw imu gyro error: week %d, sec %f, gyro %f %f %f deg/s\r\n", 
					pIlcData->week,pIlcData->imutimetarge,
					pIlcData->gyo[0]*R2D,pIlcData->gyo[1]*R2D,pIlcData->gyo[2]*R2D);
				return GILC_RET__ERR_RAW_IMU_GYRO_PARAM;
			}
		}

		if(!InitParam.iImuPeriod_ms)
		{
			static int imu_cnt = 0;
			static double imutimetarge_pre = 0;
			static double imutimetarge_diff = 0;
			static double imutimetarge_diff_total = 0;
			if(fabs(imutimetarge_pre)>1e-6)
			{
				imutimetarge_diff = ilcd.imutimetarge - imutimetarge_pre;
				if(imutimetarge_diff>0)
				{
					imutimetarge_diff_total += imutimetarge_diff;
					imu_cnt++;
					if(imu_cnt >= 100)
					{
						InitParam.iImuPeriod_ms = (int)(imutimetarge_diff_total*1000/imu_cnt+0.5);
						gilc_log("imu_period_ms get %d ms, diff_total %.3f ms, imu_num %d\r\n",InitParam.iImuPeriod_ms,imutimetarge_diff_total*1000,imu_cnt);
						imu_cnt = 0;
						imutimetarge_pre = 0;
						imutimetarge_diff = 0;
						imutimetarge_diff_total = 0;
					}
				}
			}
			imutimetarge_pre = ilcd.imutimetarge;
		}

		if(pRaw->bPPSavail)
		{
			int dTime_ms = 0;
			//int imu_period_ms = 20; /*dsf90:2018.9.20,ͬ��ʱ�����,�������20ms*/
			int imu_period_ms = s_cfgdata.imu_period_ms>InitParam.iPpsTimeErrMax_ms ?s_cfgdata.imu_period_ms: InitParam.iPpsTimeErrMax_ms;
			dTime_ms = (int)((ilcd.imutimetarge - (int)ilcd.imutimetarge)*1000);
			dTime_ms %= GNSS_PERIODE_MS;/*dsf90:Ĭ��GNSS 5Hz*/
			if(dTime_ms < imu_period_ms || dTime_ms>(GNSS_PERIODE_MS - imu_period_ms))
			{
				pIlcData->bPPSavail = pRaw->bPPSavail;
			}
			else
			{
				gilc_log("%8.6f: Update PPS flag, last flag %d, new flag %d,err times %dms\r\n", pRaw->imutimetarget, pRaw->bPPSavail, pIlcData->bPPSavail, dTime_ms);
			}
		}
		pIlcData->bMEMSavail = true;
	}
    /*******************GNSS*********************/
	if(pRaw->bGPSavail)
	{
		int iGnssSec = 0;
		double dErrTime = 0;

		iGnssSec = (int)(pRaw->gnssdata.second*1e3);
		if (fabs(iGnssSec - iGnssSec_pre - 200) > 10)
		{
			if (iGnssSec == iGnssSec_pre)
			{
			}
			else
			{
				gilc_log("GNSS RAW LOST : last %d, now %d------------\r\n", iGnssSec_pre, iGnssSec);

			}
		}
		iGnssSec_pre = iGnssSec;

		pIlcData->gpstimetarge = pRaw->gnssdata.second;
		pIlcData->week = pRaw->gnssdata.week;
		dErrTime = fabs(pRaw->gnssdata.second - pIlcData->imutimetarge);
		
#ifndef SYNC_PPS_UNUSED
		/*dsf90:ʱ��ͬ������*/
		if(!pIlcData->week || (dErrTime*1000) > MX_GNSS_DELAY_MS)
		{
			gilc_log("libgilc --- raw time error: week %d, sec %f, imu_sec %f, diff time %f s\r\n", 
				pIlcData->week,pIlcData->gpstimetarge,
				pIlcData->imutimetarge,dErrTime);
			return GILC_RET__ERR_RAW_GNSS_TIME_PARAM;
		}
#endif
		if (s_cfgdata.eGnssPosMode == GILC_GNSSPOS_MODE__LLA_DEG)
		{
			pIlcData->pos[0] = pRaw->gnssdata.lat*D2R;
			pIlcData->pos[1] = pRaw->gnssdata.lon*D2R;
		}
		else if (s_cfgdata.eGnssPosMode == GILC_GNSSPOS_MODE__LLA_RAD)
		{
			pIlcData->pos[0] = pRaw->gnssdata.lat;
			pIlcData->pos[1] = pRaw->gnssdata.lon;
		}
		else
		{
			gilc_log("libgilc --- cfg pos mode error, %d\r\n",s_cfgdata.eGnssPosMode);
			return GILC_RET__ERR_CFG_POS_MODE;
		}

#if 1  //��γ���ж��Ƿ����ظ����
		double lat = pIlcData->pos[0], lon = pIlcData->pos[1], hig = pIlcData->pos[2];
		if ((lat == 0.0&&lon == 0.0&&hig == 0.0) || ((lat == Gpospre[0]) && (lon == Gpospre[1]) && (hig == Gpospre[2])))
		{
			//gilc_log("libgilc --- gnss pos repeat error, week %d, sec %f\r\n", pIlcData->week, pIlcData->gpstimetarge);
			return GILC_RET__ERR_GNSS_POS_REPEAT;
		}
		Gpospre[0] = pIlcData->pos[0]; Gpospre[1] = pIlcData->pos[1]; Gpospre[2] = pIlcData->pos[2];
#endif	

		pIlcData->pos[2] = pRaw->gnssdata.alt;
		
		/*dsf90:��γ���������*/
		/*��γ������ 180deg���߳�����10000m*/
		if ((fabs(pIlcData->pos[0]) > PI) || (fabs(pIlcData->pos[1]) > PI) || (fabs(pIlcData->pos[2]) >= 10000))
		{
			gilc_log("libgilc --- raw pos error: week %d, sec %f, pos %f rad %f rad %f m\r\n", 
				pIlcData->week,pIlcData->gpstimetarge,
				pIlcData->pos[0],pIlcData->pos[1],pIlcData->pos[2]);
			return GILC_RET__ERR_RAW_POS_PARAM;
		}

		if(s_cfgdata.eGnssVelMode == GILC_GNSSVEL_MODE__ECEF)
		{
			double vel_ecef[3]={0};
			vel_ecef[0]=pRaw->gnssdata.vx_ecef;
			vel_ecef[1]=pRaw->gnssdata.vy_ecef;
			vel_ecef[2]=pRaw->gnssdata.vz_ecef;
			//ecef2enu(pIlcData->pos,vel_ecef,pIlcData->vn);
			pIlcData->vn[0]=vel_ecef[0];
			pIlcData->vn[1]=vel_ecef[1];
			pIlcData->vn[2]=vel_ecef[2];
			pIlcData->gnss_speed = sqrt(pow(pIlcData->vn[0],2)+pow(pIlcData->vn[1],2));
			pIlcData->heading    = atan2(pIlcData->vn[0],pIlcData->vn[1])*R2D;
			if (pIlcData->heading <0) 
				pIlcData->heading += 360;
		}
		else if(s_cfgdata.eGnssVelMode == GILC_GNSSVEL_MODE__SPEED_HEADING)
		{
			pIlcData->gnss_speed   = pRaw->gnssdata.speed;
			pIlcData->heading = pRaw->gnssdata.heading;
			pIlcData->vn[0] = pIlcData->gnss_speed*sin(pIlcData->heading*D2R);
			pIlcData->vn[1] = pIlcData->gnss_speed*cos(pIlcData->heading*D2R);
			pIlcData->vn[2] = 0;
		}
		else
		{
			gilc_log("libgilc --- cfg vel mode error, %d\r\n",s_cfgdata.eGnssVelMode);
			return GILC_RET__ERR_CFG_VEL_MODE;
		}

		
		/*dsf90:�ٶ��������*/
		/*�ٶ����ޣ�200m/s*/
		if (fabs(pIlcData->gnss_speed) > 200 || pIlcData->vn[2] > 20)
		{
			gilc_log("libgilc --- raw vel error: week %d, sec %f, vel %f %f %f m/s, speed %f m/s\r\n", 
				pIlcData->week,pIlcData->gpstimetarge,
				pIlcData->vn[0],pIlcData->vn[1],pIlcData->vn[2],pIlcData->gnss_speed);
			return GILC_RET__ERR_RAW_VEL_PARAM;
		}

		double vel_std_enu[3] = { 0 };
		double pos_std_enu[3] = { 0 };
		double heading2_std = pRaw->gnssdata.std_heading2;
		if(s_cfgdata.bGnssVelStdUse)
		{
			double vel_std_ecef[3]={0};
			vel_std_ecef[0]=pRaw->gnssdata.std_vx_ecef;
			vel_std_ecef[1]=pRaw->gnssdata.std_vy_ecef;
			vel_std_ecef[2]=pRaw->gnssdata.std_vz_ecef;
			heading2_std = 0.2;
			Var_XYZ2BLH(pIlcData->pos,vel_std_ecef,vel_std_enu);
		}
		else
		{
			if(pRaw->gnssdata.stat==4)
			{
				vel_std_enu[0]=0.05;	vel_std_enu[1]=0.05;	vel_std_enu[2]=0.1;
			}
			else if(pRaw->gnssdata.stat==5)
			{
				vel_std_enu[0]=0.1;		vel_std_enu[1]=0.1;		vel_std_enu[2]=0.2;
			}
			else
			{
				vel_std_enu[0]=0.2;		vel_std_enu[1]=0.2;		vel_std_enu[2]=0.4;
			}
			heading2_std = 0.2;

		}
		if(s_cfgdata.bGnssPosStdUse)
		{
			pos_std_enu[0] = pRaw->gnssdata.std_lat;
			pos_std_enu[1] = pRaw->gnssdata.std_lon;
			pos_std_enu[2] = pRaw->gnssdata.std_alt;
		}
		else
		{
			if(pRaw->gnssdata.stat==4)
			{
				pos_std_enu[0]=0.05;	pos_std_enu[1]=0.05;	pos_std_enu[2]=0.1;
			}
			else if(pRaw->gnssdata.stat==5)
			{
				pos_std_enu[0]=1;		pos_std_enu[1]=1;		pos_std_enu[2]=2;
			}
			else
			{
				pos_std_enu[0]=2;		pos_std_enu[1]=2;		pos_std_enu[2]=4;
			}
		}
		
		/*dsf90:��׼���������*/
		/*�ٶȱ�׼�����ޣ�0.01m/s��λ�ñ�׼�����ޣ�0.01*/
		for (int i = 0; i < 3; i++)
		{
			if(vel_std_enu[i] == 0)
			{
				gilc_log("libgilc --- raw vel std error: week %d, sec %f, vel std %f %f %f m/s\r\n", 
					pIlcData->week,pIlcData->gpstimetarge,
					vel_std_enu[0],vel_std_enu[1],vel_std_enu[2]);
				return GILC_RET__ERR_RAW_VEL_STD_PARAM;
			}
			else if (vel_std_enu[i] < 0.01)
				vel_std_enu[i] = 0.01;
				
			if(pos_std_enu[i] == 0)
			{
				gilc_log("libgilc --- raw pos std error: week %d, sec %f, pos std %f %f %f m\r\n", 
					pIlcData->week,pIlcData->gpstimetarge,
					pos_std_enu[0],pos_std_enu[1],pos_std_enu[2]);
				return GILC_RET__ERR_RAW_POS_STD_PARAM;
			}
			else if (pos_std_enu[i] < 0.01)
				pos_std_enu[i] = 0.01;
		}
		/*�����׼�����ޣ�0.2deg*/
		if (heading2_std && heading2_std <= 0.2)
		{
			//gilc_log("libgilc --- raw heading2 std error: week %d, sec %f, heading2 std %lf deg\r\n",
				//pIlcData->week, pIlcData->gpstimetarge,	heading2_std);
			//return GILC_RET__ERR_RAW_PARAM;
			heading2_std = 0.2;
		}

		pIlcData->GPV_RK[0 * 6 + 0] = SQ(vel_std_enu[0]);
		pIlcData->GPV_RK[1 * 6 + 1] = SQ(vel_std_enu[1]);
		pIlcData->GPV_RK[2 * 6 + 2] = SQ(vel_std_enu[2]);
		pIlcData->GPV_RK[3 * 6 + 3] = SQ(pos_std_enu[0] / glv.Re);
		pIlcData->GPV_RK[4 * 6 + 4] = SQ(pos_std_enu[1] / glv.Re);
		pIlcData->GPV_RK[5 * 6 + 5] = SQ(pos_std_enu[2]);

		pIlcData->stat = pRaw->gnssdata.stat;
		pIlcData->hdop = pRaw->gnssdata.hdop;
		pIlcData->age  = pRaw->gnssdata.age;
		pIlcData->ns   = pRaw->gnssdata.ns;
		pIlcData->nsused= pRaw->gnssdata.nsused;
		pIlcData->snsum  = pRaw->gnssdata.snsum;
		pIlcData->heading2 = pRaw->gnssdata.heading2 + s_cfgdata.fIns2GnssAngle[2];
		DEG_0_360(pIlcData->heading2);
		pIlcData->GA_RK[2] = SQ(heading2_std*D2R);

		/*�̶��⣬�����С���*/
		if (pIlcData->stat == 4)
		{/*��λ���Ȳ���꣬�޸Ķ�λ״̬Ϊ����*/
			if (pos_std_enu[0] > 0.1 || pos_std_enu[1] > 0.1 || pos_std_enu[2] > 0.2)
			//if (pos_std_enu[0] > 0.2 || pos_std_enu[1] > 0.2 || pos_std_enu[2] > 0.5)
			{
				pIlcData->stat = 5;
			}
		}
		if(pIlcData->stat)
			pIlcData->bGPSavail = true;
	}


	pIlcData->bValid = pIlcData->bGPSavail || pIlcData->bMEMSavail;
	return pIlcData->bValid;/*���±�־*/
}

/*add by dsf90, 2018.5.15*/
gilc_ret_e GilcProcess::GILC_LoadRaw_byStrn(char *buff,gilc_raw_t *pRaw)
{
	double val[MAXVAL]={0.0};

	if(!buff || strlen(buff) > MAXLEN)
	{
		return GILC_RET__ERR_STRN_PARAM;
	}

	pRaw->bMEMSavail=false;
	pRaw->bODOavail=false;
	pRaw->bGPSavail=false;
	pRaw->bPPSavail=false;
	if (strstr(buff,"GNSS,")|| strstr(buff, "GNSS:"))
	{
		int num = Str2Array(buff+5,",",val);
		if (num == 20)
		{
			memset(&pRaw->gnssdata, 0, sizeof(pRaw->gnssdata));
			pRaw->gnssdata.second = val[0];
			pRaw->gnssdata.week = (int)val[1];
			pRaw->gnssdata.lat = val[2];
			pRaw->gnssdata.lon = val[3];
			pRaw->gnssdata.alt = val[4];
			pRaw->gnssdata.vx_ecef = val[5];
			pRaw->gnssdata.vy_ecef = val[6];
			pRaw->gnssdata.vz_ecef = val[7];
			pRaw->gnssdata.std_lat = val[8];
			pRaw->gnssdata.std_lon = val[9];
			pRaw->gnssdata.std_alt = val[10];
			pRaw->gnssdata.std_vx_ecef = val[11];
			pRaw->gnssdata.std_vy_ecef = val[12];
			pRaw->gnssdata.std_vz_ecef = val[13];
			pRaw->gnssdata.stat = (int)val[14];
			pRaw->gnssdata.nsused = (int)val[15];
			pRaw->gnssdata.hdop = val[16];
			pRaw->gnssdata.age = val[17];
			pRaw->gnssdata.heading2 = val[20];
		}
		else if (num == 24)
		{
			memset(&pRaw->gnssdata, 0, sizeof(pRaw->gnssdata));
			pRaw->gnssdata.second = val[0];
			pRaw->gnssdata.week = (int)val[1];
			pRaw->gnssdata.lat = val[2];
			pRaw->gnssdata.lon = val[3];
			pRaw->gnssdata.alt = val[4];
			pRaw->gnssdata.vx_ecef = val[5];
			pRaw->gnssdata.vy_ecef = val[6];
			pRaw->gnssdata.vz_ecef = val[7];
			pRaw->gnssdata.std_lat = val[8];
			pRaw->gnssdata.std_lon = val[9];
			pRaw->gnssdata.std_alt = val[10];
			pRaw->gnssdata.std_vx_ecef = val[11];
			pRaw->gnssdata.std_vy_ecef = val[12];
			pRaw->gnssdata.std_vz_ecef = val[13];
			pRaw->gnssdata.pos_type = (int)val[14];
			pRaw->gnssdata.nsused = (int)val[15];
			pRaw->gnssdata.hdop = val[16];
			pRaw->gnssdata.age = val[17];
			pRaw->gnssdata.heading2 = val[18];
			pRaw->gnssdata.ns = (int)val[19];
			pRaw->gnssdata.snsum = (int)val[20];
			pRaw->gnssdata.pdop = val[21];
			pRaw->gnssdata.vdop = val[22];
			pRaw->gnssdata.std_heading2 = val[23];
		}
		else if (num == 35 || num == 40)
		{
			memset(&pRaw->gnssdata, 0, sizeof(pRaw->gnssdata));
			pRaw->gnssdata.second      = val[0];
			pRaw->gnssdata.week        = (int)val[1];
			pRaw->gnssdata.lat         = val[2];
			pRaw->gnssdata.lon         = val[3];
			pRaw->gnssdata.alt         = val[4];
			pRaw->gnssdata.vx_ecef     = val[5];
			pRaw->gnssdata.vy_ecef     = val[6];
			pRaw->gnssdata.vz_ecef     = val[7];
			pRaw->gnssdata.std_lat     = val[8];
			pRaw->gnssdata.std_lon     = val[9];
			pRaw->gnssdata.std_alt     = val[10];
			pRaw->gnssdata.std_vx_ecef = val[11];
			pRaw->gnssdata.std_vy_ecef = val[12];
			pRaw->gnssdata.std_vz_ecef = val[13];
			pRaw->gnssdata.speed       = val[14];
			pRaw->gnssdata.heading     = val[15];
			pRaw->gnssdata.stat        = (int)val[16];
			pRaw->gnssdata.age         = val[17];
			pRaw->gnssdata.hdop        = val[18];
			pRaw->gnssdata.pdop        = val[19];
			pRaw->gnssdata.vdop        = val[20];
			pRaw->gnssdata.tdop        = val[21];
			pRaw->gnssdata.gdop        = val[22];
			pRaw->gnssdata.ns          = (int)val[23];
			pRaw->gnssdata.nsused      = (int)val[24];
			pRaw->gnssdata.snsum       = (int)val[25];
			pRaw->gnssdata.ns2         = (int)val[26];
			pRaw->gnssdata.nsused2     = (int)val[27];
			pRaw->gnssdata.baseline    = val[28];
			pRaw->gnssdata.heading2    = (val[29]);
			pRaw->gnssdata.std_heading2= val[30];
			pRaw->gnssdata.pitch2      = val[31];
			pRaw->gnssdata.std_pitch2  = val[32];
			pRaw->gnssdata.pos_type    = (int)val[33];
			pRaw->gnssdata.pos_type2   = (int)val[34];
			if (num == 40)
			{
				pRaw->gnssdata.sol_status     = (int)val[35];
				pRaw->gnssdata.ext_sol_status = (int)val[36];
				pRaw->gnssdata.time_status    = (int)val[37];
				pRaw->gnssdata.gnss_used_flag = (int)val[38];
				pRaw->gnss_delay_ms           = (int)val[39];
			}
		}
		else
		{
			gilc_log("Unknow gnss data num %d\r\n",num);
			return GILC_RET__ERR_STRN_PARAM;
		}
		
		pRaw->bGPSavail =true;
		return GILC_RET__LOAD_STRN_GNSS;
	}
	else if (strstr(buff,"IMU,"))
	{
		int num = Str2Array(buff+4,",",val);
		if(num == 9 || num == 10 || num == 11)
		{
			memset(&pRaw->memsdate, 0, sizeof(pRaw->memsdate));
			pRaw->imutimetarget = val[0];
			pRaw->memsdate.alldata[0] = val[1];
			pRaw->memsdate.alldata[1] = val[2];
			pRaw->memsdate.alldata[2] = val[3];
			pRaw->memsdate.alldata[3] = val[4];
			pRaw->memsdate.alldata[4] = val[5];
			pRaw->memsdate.alldata[5] = val[6];
			pRaw->bPPSavail=((int)val[8]>0?true:false);// ���ݱ�־λ�ж�PPS
			if (num == 10)
				pRaw->memsdate.temper = val[9];
		}
		else
		{
			gilc_log("Unknow imu data num %d\r\n",num);
			return GILC_RET__ERR_STRN_PARAM;
		}
		pRaw->bMEMSavail = true;
		return GILC_RET__LOAD_STRN_IMU;
	}
	else if (strstr(buff,"ODO,"))
	{
		int num = Str2Array(buff+4,",",val);
		if(num == 10)
		{	
			memset(&pRaw->memsdate, 0, sizeof(pRaw->memsdate));
			pRaw->ododata.wheel_heading        = val[1];		
			pRaw->ododata.wheel_vel            = val[2];		
			pRaw->ododata.wheel_vel_left       = val[3];		
			pRaw->ododata.wheel_vel_right      = val[4];		
			pRaw->ododata.wheel_distance       = val[5];
			pRaw->ododata.wheel_heading_std    = val[6];		
			pRaw->ododata.wheel_vel_std        = val[7];	
			pRaw->ododata.wheel_mode           = (int)val[8];
			pRaw->ododata.use_flag             = (int)val[9];
		}
		else
		{
			gilc_log("Unknow odo data num %d\r\n",num);
			return GILC_RET__ERR_STRN_PARAM;
		}
		pRaw->bODOavail = true;
		return GILC_RET__LOAD_STRN_ODO;
	}
	
	return GILC_RET__ERR_STRN_UNKNOW;
}

gilc_ret_e GilcProcess::GILC_PROCESS_Vehicle(gilc_raw_t* pstRaw,gilc_result_t* pstOut)
{	
	gilc_ret_e ret = GILC_RET__RST_NOTHING;
/******************************ԭʼ������������**********************************/
	if(pstRaw->bGPSavail && pstRaw->gnssdata.second != stRaw_tmp.gnssdata.second)
	{
		GILC_GnssRaw_Correct(pstRaw);//gnss��λ״̬����		
		stRaw_tmp.bGPSavail = pstRaw->bGPSavail;
		stRaw_tmp.gnssdata = pstRaw->gnssdata;
		stRaw_tmp.gnss_delay_ms = pstRaw->gnss_delay_ms;
	}

	if(pstRaw->bMEMSavail)
	{
		GILC_ImuAxis_Correct(pstRaw);//imu��ϵ���� ENU����ǰ�ϣ�	
		stRaw_tmp.bMEMSavail = pstRaw->bMEMSavail;
		stRaw_tmp.memsdate = pstRaw->memsdate;
		stRaw_tmp.imutimetarget = pstRaw->imutimetarget;
	}


	if (pstRaw->bPPSavail)
	{
		stRaw_tmp.bPPSavail = pstRaw->bPPSavail;
	}
/************************************ԭʼ���ݡ���Ͻ���ṹ�帳ֵ******************************************/	
	ret = (gilc_ret_e)GILC_Raw2CLCData(pstRaw,&ilcd);
	
	if(ret<0) 
		return ret;
	else if (ret == 0)
		return GILC_RET__RST_NOTHING;

	if (++raw_cnt % (100 * 60) == 0)
	{
		gilc_log("read imu num %d \r\n", raw_cnt);
	}
	
	gtime_t gt= gpst2time(ilcd.week, ilcd.imutimetarge);
	time2epoch(gpst2utc(gt), ilcd.ep);Mequalm(ilcd.ep,6,1,glv.ep);

#ifdef GILC_DEBUG_FILE
	if(!debugfile.bInit && s_cfgdata.debug_level >= 1)
	{
#ifdef SYNC_PPS_UNUSED
		if(ilcd.imutimetarge>0)
#else
		if ((ilcd.week>0) && (ilcd.imutimetarge>0))
#endif
		{
			char * pcOutPath = NULL;
			char * pcTmpPath = NULL;
			if(s_cfgdata.bFilePathCfgUse)
			{
				if(s_cfgdata.debug_outfile_path[0])
					pcOutPath = s_cfgdata.debug_outfile_path;
				if(s_cfgdata.debug_tmpfile_path[0])
					pcTmpPath = s_cfgdata.debug_tmpfile_path;			
			}
			else
			{
				pcOutPath = (char *)TEST_OUT_FILE_PATH;
				pcTmpPath = (char *)TEST_TMP_FILE_PATH;			
			}
			debugfile.Init(pcOutPath,!s_cfgdata.bOutFileSaveClose,pcTmpPath,!s_cfgdata.bTmpFileSaveClose);
			gilc_log(cGilcInitMsg);
		}
	}	
#endif
/************************************ԭʼ���ݵ�ͨ�˲�******************************************/
	/*add by dsf90,2018.6.8: IIR ʵʱ�˲���*/
	double imu_raw[6];
	double imu_iir[6];
	Mequalm(ilcd.acc,3,1,&imu_raw[0]);
	Mequalm(ilcd.gyo,3,1,&imu_raw[3]);
	iir_filter.IIR_Lowpass(imu_iir,imu_raw);
	Mequalm(&imu_iir[0],3,1,ilcd.acc_iir);
	Mequalm(&imu_iir[3],3,1,ilcd.gyo_iir);
	//Mequalm(&imu_iir[0],3,1,ilcd.acc);
	//Mequalm(&imu_iir[3],3,1,ilcd.gyo);

	debugfile.SaveRaw(&ilcd);

	if(!GetGPS)
	{
		if(ilcd.bGPSavail)
		{
			GetGPS=true;
		}
		ret = GILC_RET__RST_INITING;
	}
/***************************************���˴�����̬�жϡ�ǰ������ٶ��ж�*****************************************/
	if(GetGPS)
	{
		int iStatic = 0;
			iStatic = gipro.detect.DetectStatic_car_dsf(ilcd.ep,ilcd.acc,ilcd.gyo,ilcd.vn,gipro.ins.vm_car,ilcd.bGPSavail,gipro.bGnssLost,ilcd.stat);
			//kinalign.bStatic = gipro.bStatic = iStatic;
			if (ilcd.bGPSavail)
			{
				if (Raw_gnss_speed.size() < 5)
				{
					Raw_gnss_speed.push_back(ilcd.gnss_speed);
				}
				else
				{
					Raw_gnss_speed.erase(Raw_gnss_speed.begin());//���ڻ���
					Raw_gnss_speed.push_back(ilcd.gnss_speed);
					double mean_speed = GetAveStd(Raw_gnss_speed, 0);//���㴰�ھ�ֵ 
					if (ilcd.stat == 4)
					{
						if (mean_speed < 0.02)
						{
							gipro.bgnssStatic = true;
							//gnss_static = 1;
						}
						else
						{
							gipro.bgnssStatic = false;
							//gnss_static = 0;
						}
					}
					//else
					//{
					//	if (mean_speed < 0.1)
					//	{
					//		gipro.bgnssStatic = true;
					//		//gnss_static = 1;
					//	}
					//	else
					//	{
					//		gipro.bgnssStatic = false;
					//		//gnss_static = 0;
					//	}
					//}

				}
			}
			if(ilcd.bMEMSavail)
			{
				if (Rst_speed.size() < 20)
				{
					Rst_speed.push_back(Rst_speed_pre);
				}
				else
				{
					Rst_speed.erase(Rst_speed.begin());//���ڻ���
					Rst_speed.push_back(Rst_speed_pre);
					double mean_speed = GetAveStd(Rst_speed, 0);//���㴰�ھ�ֵ
					if (mean_speed < 0.05)
					{
						gilc_static = 1;
					}
					else
					{
						gilc_static = 0;
					}
				}
			}
			//if (gilc_static || iStatic)
			if (gnss_static)
			{
				kinalign.bStatic = gipro.bStatic = 0;
			}
			else
			{
				kinalign.bStatic = gipro.bStatic = 0;
			}
		if (raw_cnt % (100*6) == 0)
			{
				//gilc_log("gnss:%d  gilc:%d\r\n", iStatic, gilc_static);
				if (gilc_static == 1)
				{
					gilc_log("%f ----in static by gilc_speed-------\r\n", ilcd.imutimetarge);
				}
				else
				{
					gilc_log("%f ----out static by gilc_speed-------\r\n", ilcd.imutimetarge);
				}
			}
		if(!kinalign.dyni.bStaticStd)
		{
			/*dsf90:�޸�Detect()����Ϊ:��һ���㾲̬std*/
			kinalign.dyni.Detect(ilcd.gyo,ilcd.acc);
		}

		if(!bAlign)
		{
			if(s_cfgdata.bEkfXUse)
			{
				if(s_cfgdata.stEkfX_Init.DualAntErr)
				{/*˫���ߺ�������ѱ궨*/
					gipro.dDualAntYawBias = s_cfgdata.stEkfX_Init.DualAntErr;
					gipro.bDualAntCalibrateOk = 1;//�����ļ��궨�ɹ�1 
				}
				
				if(s_cfgdata.stEkfX_Init.WheelHeadingScale && s_cfgdata.stEkfX_Init.Kwh)
				{/*������ת�Ǳ���ϵ����ƫ���ѱ궨*/
					gipro.ins.WheelHeadingScale = s_cfgdata.stEkfX_Init.WheelHeadingScale;
					gipro.ins.Kwh = s_cfgdata.stEkfX_Init.Kwh;
					gipro.ins.Bwh = s_cfgdata.stEkfX_Init.Bwh;
					gipro.bWheelHeadingCalibrateOk = 1;
				}
				gipro.iGilcRunMode = 0;/*don't need calibrate*/
				gipro.dInitTimesMin = 5 *60;//
				
			}
			else
			{
				gipro.iGilcRunMode = 1; /*calibrate first*/
				if (pstRaw->gnssdata.stat == 4)
				{
					gipro.dInitTimesMin = 3 * 60.0;
				}
				else
				{
					gipro.dInitTimesMin = 4 * 60.0;
				}
			}
			
			bool align=kinalign.KinmateAlign(ilcd,gipro);
			if (align)
			{
				bAlign=true;
				
				gilc_log("%02d:%02d:%06.3f start: %f\r\n",
					int(ilcd.ep[3]),int(ilcd.ep[4]),ilcd.ep[5],ilcd.imutimetarge);
	
				gipro.para.setdavp(kinalign.Pgposvn, 5, 2);
				gipro.para.setpobs(kinalign.Pgposvn);
				double kf_Q_init_att[3] = {0.0};
				double kf_Q_init_vel[3] = {0.0};
				double kf_Q_init_pos[3] = {0.0};
				double kf_Q_init_gyobias[3] = {0.0};
				double kf_Q_init_accbias[3] = {0.0};
				
				double kf_P_init_gyobias[3]    = { InitParam.dKfInitP_GyroBais*glv.deg,InitParam.dKfInitP_GyroBais*glv.deg,InitParam.dKfInitP_GyroBais*glv.deg };
				double kf_P_init_accbias[3]    = { InitParam.dKfInitP_AccBais*glv.g0,InitParam.dKfInitP_AccBais*glv.g0,InitParam.dKfInitP_AccBais*glv.g0 };
				double kf_P_init_installerr[3] = { InitParam.dKfInitP_InstallErr,InitParam.dKfInitP_InstallErr,InitParam.dKfInitP_InstallErr};
				double kf_P_init_lever[3]      = { InitParam.dKfInitP_Lever,InitParam.dKfInitP_Lever,InitParam.dKfInitP_Lever/10};
				double gww[3],aww[3],gs[3],as[3];			
				if(s_cfgdata.bStdCfgUse)
				{
					gww[0] = s_cfgdata.gyro_std[0]*glv.deg;
					gww[1] = s_cfgdata.gyro_std[1]*glv.deg;
					gww[2] = s_cfgdata.gyro_std[2]*glv.deg;
					aww[0] = s_cfgdata.accle_std[0]*glv.g0;
					aww[1] = s_cfgdata.accle_std[1]*glv.g0;
					aww[2] = s_cfgdata.accle_std[2]*glv.g0;
				}
				else
				{
					gww[0] = GILC_GYRO_STD_X*glv.deg;
					gww[1] = GILC_GYRO_STD_Y*glv.deg;
					gww[2] = GILC_GYRO_STD_Z*glv.deg;
					aww[0] = GILC_ACC_STD_X*glv.g0;
					aww[1] = GILC_ACC_STD_Y*glv.g0;
					aww[2] = GILC_ACC_STD_Z*glv.g0;
				}
				if(s_cfgdata.bStdCfgUse)
				{
					gs[0] = s_cfgdata.gyro_walk[0]*glv.dpsh;
					gs[1] = s_cfgdata.gyro_walk[1]*glv.dpsh;
					gs[2] = s_cfgdata.gyro_walk[2]*glv.dpsh;
					as[0] = s_cfgdata.vel_walk[0]*glv.ugpsh;
					as[1] = s_cfgdata.vel_walk[1]*glv.ugpsh;
					as[2] = s_cfgdata.vel_walk[2]*glv.ugpsh;
				}
				else
				{
					gs[0] = gs[1] = gs[2] = GILC_GYRO_WALK*glv.dpsh;
					as[0] = as[1] = as[2] = GILC_VEL_WALK*glv.ugpsh;
				}
/*******************************INS�����ʼλ�á���̬���ٶ�װ��********************************/					
				gipro.para.setimu(kf_P_init_gyobias,kf_P_init_accbias,gww,aww,gs,as,3);
				if(s_cfgdata.bEkfXUse)
				{
					gipro.ins.Init(kinalign.Att,kinalign.VnL,kinalign.PosL,s_cfgdata.stEkfX_Init.GyoBias,s_cfgdata.stEkfX_Init.AccBias,s_cfgdata.stEkfX_Init.Lever,dGnss2OutPointLever, s_cfgdata.stEkfX_Init.InstallErr);
					gilc_log("cfg GyoBias:%f,%f,%f,AccBias:%f,%f,%f,InstallErr:%f,%f,%f\r\n",
						s_cfgdata.stEkfX_Init.GyoBias[0], s_cfgdata.stEkfX_Init.GyoBias[1], s_cfgdata.stEkfX_Init.GyoBias[2],
						s_cfgdata.stEkfX_Init.AccBias[0], s_cfgdata.stEkfX_Init.AccBias[1], s_cfgdata.stEkfX_Init.AccBias[2],
						s_cfgdata.stEkfX_Init.InstallErr[0], s_cfgdata.stEkfX_Init.InstallErr[1], s_cfgdata.stEkfX_Init.InstallErr[2]);
					for (int i = 0;i < 3;i++)
					{
						gipro.cfg_install_anger[i] = s_cfgdata.stEkfX_Init.InstallErr[i];
					}
				}
				else
				{
					double lever[3] = {s_cfgdata.fIns2GnssVector[0],s_cfgdata.fIns2GnssVector[1],s_cfgdata.fIns2GnssVector[2]};
					kinalign.PRY_Install[0] = 0* D2R;
					gipro.ins.Init(kinalign.Att,kinalign.VnL,kinalign.PosL,eb,db,lever,dGnss2OutPointLever,kinalign.PRY_Install);
				}
				
				if(kinalign.Att[2])
				{
					/*�����ѳ�ʼ����ͨ���˱ۡ���̬�����Ƽ���IMU��ʼλ��*/
					double dLeverAnt2Ins[3] = { 0 };
					Mmuln(s_cfgdata.stEkfX_Init.Lever, 3, 1, -1, dLeverAnt2Ins);
					gipro.ins.Lever(kinalign.PosL, kinalign.VnL, gipro.ins.pos, gipro.ins.vn, dLeverAnt2Ins);
				}
				else
				{
					/*����δ��ʼ����������λ����ΪIMU��ʼλ��*/
					Mequalm(kinalign.VnL, 3, 1, gipro.ins.vn);
					Mequalm(kinalign.PosL, 3, 1, gipro.ins.pos);
				}
				
				double cbn[9], vb[3];
				a2mat(gipro.ins.PRY_Install, gipro.ins.Cmb);
				a2mat(gipro.ins.att, gipro.ins.Cnb);
				Mtn(gipro.ins.Cnb, 3, 3, cbn);
				Mtn(gipro.ins.Cmb, 3, 3, gipro.ins.Cbm);
				Mmulnm(cbn, gipro.ins.vn, 3, 3, 1, vb);
				Mmulnm(gipro.ins.Cmb, vb, 3, 3, 1, gipro.ins.vm_car);
				Mmulnm(gipro.ins.Cnb, gipro.ins.Cbm, 3, 3, 3, gipro.ins.Cnm);
				m2att(gipro.ins.Cnm, gipro.ins.att_car);

				gipro.preheading=gipro.ins.att[2];
				gpos_t gpostemp;
				equalgpos(&gpostemp,&ilcd);
				gipro.pregpos.push_back(gpostemp);
				Mequalm(ilcd.gyo,3,1,gipro.wmpre);
				Mequalm(ilcd.acc,3,1,gipro.vmpre);
				gipro.tpre=ilcd.imutimetarge-0.01;

				memset(gipro.kf.kf_Q_init,0,sizeof(gipro.kf.kf_Q_init));
				memset(gipro.kf.kf_P_init, 0, sizeof(gipro.kf.kf_P_init));

				memcpy(kf_Q_init_att,&gww,sizeof(gww));
				memcpy(kf_Q_init_vel,&aww,sizeof(aww));
				//kf_Q_init_pos[0] = glv.mpsh/glv.Re;
				//kf_Q_init_pos[1] = glv.mpsh/glv.Re;
				//kf_Q_init_pos[2] = glv.mpsh;			
				memcpy(kf_Q_init_gyobias,&gs,sizeof(gs));
				memcpy(kf_Q_init_accbias,&as,sizeof(as));
				for (int i=0;i<3;i++)
				{
					gipro.kf.kf_Q_init[i+ 0]=(kf_Q_init_att[i]);	
					gipro.kf.kf_Q_init[i+ 3]=(kf_Q_init_vel[i]);	
					gipro.kf.kf_Q_init[i+ 6]=(kf_Q_init_pos[i]);	
					gipro.kf.kf_Q_init[i+ 9]=(kf_Q_init_gyobias[i]);		 
					gipro.kf.kf_Q_init[i+12]=(kf_Q_init_accbias[i]);		 		 
				}	
				if(NUMX>=25)
					gipro.kf.kf_Q_init[24]= 0.5;				 
				
				double kf_P_init_scale = 1.0;
				double kf_P_init_scale_lever = 1.0;

				if(s_cfgdata.fIns2GnssVector[0] || s_cfgdata.fIns2GnssVector[1] || s_cfgdata.fIns2GnssVector[2])
					kf_P_init_scale_lever = 0.25;
				
				if(s_cfgdata.bEkfXUse)
				{
					kf_P_init_scale = 0;
					kf_P_init_scale_lever = 0;
				}

				for (int i=0;i<3;i++)
				{
					gipro.kf.kf_P_init[i+ 0]=(gipro.para.davp[i]);	
					gipro.kf.kf_P_init[i+ 3]=(gipro.para.davp[i+3]);	
					gipro.kf.kf_P_init[i+ 6]=(gipro.para.davp[i+6]);	
					gipro.kf.kf_P_init[i+ 9]=(kf_P_init_gyobias[i]);		 
					gipro.kf.kf_P_init[i+12]=(kf_P_init_accbias[i]);		 
					gipro.kf.kf_P_init[i+15]=(kf_P_init_installerr[i]*kf_P_init_scale);		 
					gipro.kf.kf_P_init[i+18]=(kf_P_init_lever[i]* kf_P_init_scale_lever);
				}
			}
			ret = GILC_RET__RST_INITING;
		}
/******************************��ʼ��׼��ɣ������㷨����********************************/
		if(bAlign)
		{
			int stat = 0;
			//double boat_yaw = gipro.ins.att_car[2] * R2D;//��ͷ����
			double boat_yaw = -atan2(gipro.ins.vnL[0], gipro.ins.vnL[1])*R2D;
			gipro.ins.yaw_heading_err = -ilcd.heading2 - boat_yaw;
			gipro.ins.imutimetarge = ilcd.imutimetarge;
			stat=gipro.GIPro_P2(ilcd);
			for (int i = 0;i < 3;i++)
			{
				gipro.ins.pre_ver[i] = gipro.ins.vn[i];
			}
			
			if(ilcd.bMEMSavail)	
			{
				if(fabs(gipro.ins.vm_car[1])<0.02)
					gipro.iDriverMode = 1;
				else
					gipro.iDriverMode = 0;
			}
			//GILC_Rst_Vel_smooth(&gipro);//�ٶȷ���ƽ��
/***********************************debug����м�������*****************************************/
			if(!s_cfgdata.bOutFileSaveClose)
			{
				debugfile.SaveRst(stat,&gipro,&ilcd);
			}

			if(ilcd.bGPSavail)	
			{
				pstOut->bPPSSync = gipro.bPPSSync;
				if(pstOut->bPPSSync)
				{			
					Mequalm(gipro.dposb_sync,3,1,stRstInner.dpos_sync);
				}	
				return GILC_RET__RST_NOTHING;
			}

			if(InitParam.iImuPeriod_ms)
			{
				dbg_cnt++;
				if(dbg_cnt %(10*1000/InitParam.iImuPeriod_ms)==0)/*10Hz log*/
				{
					dbg_cnt = 0;
					gilc_log("%f bais %f,%f,%f,%f,%f,%f  ",
						ilcd.imutimetarge,
						gipro.ins.eb[0],
						gipro.ins.eb[1],
						gipro.ins.eb[2],
						gipro.ins.db[0],
						gipro.ins.db[1],
						gipro.ins.db[2]);
					gilc_log("install %f,%f,%f,%f,%f,%f dul_heading_err:%f\r\n",
						gipro.ins.PRY_Install[0]*R2D,
						gipro.ins.PRY_Install[1]*R2D,
						gipro.ins.PRY_Install[2]*R2D,
						gipro.ins.lever[0],
						gipro.ins.lever[1],
						gipro.ins.lever[2],
						gipro.dDualAntYawBias
						);
				}
			}
			if (gipro.bGnssLost)
				ilcd.stat = 3;

			if(gipro.bInstallOk)
			{
				if (gipro.bGnssLost)
					ret = GILC_RET__RST_INS_MODE;/*ins mode*/
				else
					ret = GILC_RET__RST_STABLE;
			}
			else
			{
				//if(gipro.iInitTimes_ms >= 20*60*1000)
				//	ret = GILC_RET__ERR_CALIBRATE_FAIL;
				//else
					ret = GILC_RET__RST_RESOLVING;
			}
		}
	}

	bool bDualAntAvail = 0;
	if (ilcd.heading2 && ilcd.GA_RK[2] && ilcd.GA_RK[2]<15*D2R)
	{
		bDualAntAvail = 1;
		pstOut->bHeadingOk = true;
	}
	
	gilc_ret_e iGilcStatus = ret; 
/*************************************��Ͻ��״̬�ж�******************************************/
//��װ���궨�ɹ�+˫���߱궨�ɹ�==iGilcStatus = GILC_RET__RST_STABLE
//gnss�жϣ�iGilcStatus = GILC_RET__RST_INS_MODE
	if(!gipro.iGilcRunMode)//iGilcRunMode=0,����Ҫ�궨
	{
		if(iGilcStatus >= GILC_RET__RST_RESOLVING)
		{/*��������ģʽ���Ѿ��궨�������״̬-���ģʽ*/
			if (gipro.bGnssLost)
				iGilcStatus = GILC_RET__RST_INS_MODE;/*ins mode*/
			else
				iGilcStatus = GILC_RET__RST_STABLE;
		}
	}
	else//iGilcRunMode=1,��Ҫ�궨
	{/*�궨ģʽ���ȴ�˫���߱궨���*/
		if(gipro.bDualAntAvail)
		{
			if(!gipro.bDualAntCalibrateOk && iGilcStatus > GILC_RET__RST_RESOLVING)
			{
				iGilcStatus = GILC_RET__RST_RESOLVING;
			}
		}
	}
	//printf("iGilcStatus=%d\r\n", iGilcStatus);
	GILC_Update_Status(pstOut,iGilcStatus,ilcd.stat,bDualAntAvail);
	pstOut->iSensorFlag &= 0xFFFFFF00;
/*************************************��Ͻ������*****************************************/
//λ�ã�deg  �߳� ��m
//��̬��deg  ��̬����-180~180deg����ƫ����  ˫���ߺ���0~360deg����ƫ����
//���ٶȣ�������װ����ƫ���� deg
//���ٶȣ�������װ����ƫ���� g
	if (ilcd.bMEMSavail||ilcd.bGPSavail)
	{
		if (iGilcStatus >= GILC_RET__RST_RESOLVING)
		{
			if (!gipro.bOdoLost)
			{
				if (gipro.bWheelVelCalibrateOk)
					pstOut->iSensorFlag |= GILC_SENSOR_USE__BODY_SPEED;
				if (gipro.bWheelHeadingCalibrateOk)
					pstOut->iSensorFlag |= GILC_SENSOR_USE__BODY_HEADING;
			}
			if (!gipro.bGnssLost)
				pstOut->iSensorFlag |= GILC_SENSOR_USE__GNSS;

			pstOut->iSensorFlag |= GILC_SENSOR_USE__IMU;

			pstOut->second = ilcd.imutimetarge;
			pstOut->week = ilcd.week;

			pstOut->pitch = gipro.ins.att_car[0] * R2D;
			pstOut->roll = gipro.ins.att_car[1] * R2D;
			pstOut->yaw = gipro.ins.att_car[2] * R2D;

			if (s_cfgdata.iOutReferPoint == GILC_OUTREFER_POINT__GNSS)
			{
				pstOut->lla[0] = gipro.ins.posL[0] * R2D;
				pstOut->lla[1] = gipro.ins.posL[1] * R2D;
				if (pstRaw->gnssdata.stat == 4)
				{
					pstOut->lla[2] = pstRaw->gnssdata.alt;
					GILC_Rst_Filter(pstOut);//�߳�����ƽ��
				}
				else
				{
					pstOut->lla[2] = gipro.ins.posL[2];
				}
				if (gipro.bgnssStatic)//gnss�̶�ʱ����̬���gnssλ��
				{
					pstOut->lla[0] = ilcd.pos[0] * R2D;
					pstOut->lla[1] = ilcd.pos[1] * R2D;
					pstOut->lla[2] = ilcd.pos[2];
				}
				/*gnssλ��ת��������λ��*/
				double lever_gnss_to_sounder[3] = { 0,0,0 };
				double pos_sounder[3] = { 0 };//�۸�ֵ
				gipro.ins.Lever_gnss_to_sounder(pstOut->lla, pos_sounder, lever_gnss_to_sounder);
				//pstOut->lla[2] = pos_sounder[2];//������Ϊ���ĵĸ߳�
				memcpy(pstOut->vel_enu, gipro.ins.vnL, sizeof(gipro.ins.vnL));

				//pstOut->speed = sqrt(SQ(pstOut->vel_enu[0]) + SQ(pstOut->vel_enu[1]));
				pstOut->speed = sqrt(SQ(gipro.ins.vn[0]) + SQ(gipro.ins.vn[1]));

				pstOut->heading = -pstOut->yaw;
				if (pstOut->heading < 0) pstOut->heading += 360;
			}
			else if (s_cfgdata.iOutReferPoint == GILC_OUTREFER_POINT__IMU)
			{
				pstOut->lla[0] = gipro.ins.pos[0] * R2D;
				pstOut->lla[1] = gipro.ins.pos[1] * R2D;
				pstOut->lla[2] = gipro.ins.pos[2];
				memcpy(pstOut->vel_enu, gipro.ins.vn, sizeof(gipro.ins.vn));
				pstOut->speed = sqrt(SQ(gipro.ins.vn[0]) + SQ(gipro.ins.vn[1]));

				pstOut->heading = -pstOut->yaw;
				if (pstOut->heading < 0) pstOut->heading += 360;
			}
			else if (s_cfgdata.iOutReferPoint == GILC_OUTREFER_POINT__INPUT)
			{

				pstOut->lla[0] = gipro.ins.posL2[0] * R2D;
				pstOut->lla[1] = gipro.ins.posL2[1] * R2D;
				pstOut->lla[2] = gipro.ins.posL2[2];
				memcpy(pstOut->vel_enu, gipro.ins.vnL2, sizeof(gipro.ins.vnL2));
				pstOut->speed = sqrt(SQ(gipro.ins.vnL2[0]) + SQ(gipro.ins.vnL2[1]));
				pstOut->heading = -pstOut->yaw;
				if (pstOut->heading < 0) pstOut->heading += 360;
			}
			//GILC_Rst_Filter(pstOut);//�����ٶ�ƽ��
			pstOut->std_pry[0] = sqrt(gipro.kf.Pxk[0 * gipro.kf.ROW + 0])*R2D;
			pstOut->std_pry[1] = sqrt(gipro.kf.Pxk[1 * gipro.kf.ROW + 1])*R2D;
			pstOut->std_pry[2] = sqrt(gipro.kf.Pxk[2 * gipro.kf.ROW + 2])*R2D;

			pstOut->std_vel[0] = sqrt(gipro.kf.Pxk[3 * gipro.kf.ROW + 3]);
			pstOut->std_vel[1] = sqrt(gipro.kf.Pxk[4 * gipro.kf.ROW + 4]);
			pstOut->std_vel[2] = sqrt(gipro.kf.Pxk[5 * gipro.kf.ROW + 5]);
			pstOut->std_speed = sqrt(SQ(pstOut->std_vel[0]) + SQ(pstOut->std_vel[1]) + SQ(pstOut->std_vel[2]));

			pstOut->std_lla[0] = sqrt(gipro.kf.Pxk[6 * gipro.kf.ROW + 6])*RE_WGS84;
			pstOut->std_lla[1] = sqrt(gipro.kf.Pxk[7 * gipro.kf.ROW + 7])*RE_WGS84;
			pstOut->std_lla[2] = sqrt(gipro.kf.Pxk[8 * gipro.kf.ROW + 8]);

			pstOut->bHeadingOk = true;

			pstOut->acc_car[0] = gipro.ins.am[0] / glv.g0;
			pstOut->acc_car[1] = gipro.ins.am[1] / glv.g0;
			pstOut->acc_car[2] = gipro.ins.am[2] / glv.g0;

			pstOut->acc[0] = gipro.ins.fm[0] / glv.g0;
			pstOut->acc[1] = gipro.ins.fm[1] / glv.g0;
			pstOut->acc[2] = gipro.ins.fm[2] / glv.g0;

			pstOut->gyro[0] = gipro.ins.wim[0] * R2D;
			pstOut->gyro[1] = gipro.ins.wim[1] * R2D;
			pstOut->gyro[2] = gipro.ins.wim[2] * R2D;

			stEkfX.Att[0] = gipro.ins.att[0];
			stEkfX.Att[1] = gipro.ins.att[1];
			stEkfX.Att[2] = gipro.ins.att[2];

			stEkfX.Vel[0] = gipro.ins.vn[0];
			stEkfX.Vel[1] = gipro.ins.vn[1];
			stEkfX.Vel[2] = gipro.ins.vn[2];

			stEkfX.Pos[0] = gipro.ins.pos[0];
			stEkfX.Pos[1] = gipro.ins.pos[1];
			stEkfX.Pos[2] = gipro.ins.pos[2];

			stEkfX.GyoBias[0] = gipro.ins.eb[0];
			stEkfX.GyoBias[1] = gipro.ins.eb[1];
			stEkfX.GyoBias[2] = gipro.ins.eb[2];

			stEkfX.AccBias[0] = gipro.ins.db[0];
			stEkfX.AccBias[1] = gipro.ins.db[1];
			stEkfX.AccBias[2] = gipro.ins.db[2];

			stEkfX.InstallErr[0] = gipro.ins.PRY_Install[0];
			stEkfX.InstallErr[1] = gipro.ins.PRY_Install[1];
			stEkfX.InstallErr[2] = gipro.ins.PRY_Install[2];

			stEkfX.Lever[0] = gipro.ins.lever[0];
			stEkfX.Lever[1] = gipro.ins.lever[1];
			stEkfX.Lever[2] = gipro.ins.lever[2];

			stEkfX.WheelHeadingScale = gipro.ins.WheelHeadingScale;
			stEkfX.Kwh = gipro.ins.Kwh;
			stEkfX.Bwh = gipro.ins.Bwh;
			stEkfX.Kd = gipro.ins.Kd;
			stEkfX.DualAntErr = gipro.dDualAntYawBias;
		}
		else
		{
			pstOut->iSensorFlag = GILC_SENSOR_USE__GNSS;

			pstOut->second = ilcd.imutimetarge;
			pstOut->week = ilcd.week;

			pstOut->acc[0] = pstRaw->memsdate.accel[0];
			pstOut->acc[1] = pstRaw->memsdate.accel[1];
			pstOut->acc[2] = pstRaw->memsdate.accel[2];

			pstOut->gyro[0] = pstRaw->memsdate.gyro[0];
			pstOut->gyro[1] = pstRaw->memsdate.gyro[1];
			pstOut->gyro[2] = pstRaw->memsdate.gyro[2];

			pstOut->lla[0] = ilcd.pos[0] * R2D;
			pstOut->lla[1] = ilcd.pos[1] * R2D;
			pstOut->lla[2] = ilcd.pos[2];

			memcpy(&pstOut->vel_enu[0], &ilcd.vn[0], sizeof(ilcd.vn));
			pstOut->speed = sqrt(SQ(ilcd.vn[0]) + SQ(ilcd.vn[1]));


			pstOut->heading = ilcd.heading2;
			if (pstOut->heading < 0) pstOut->heading += 360;
			pstOut->pitch = atan2(ilcd.acc[1], sqrt(pow(ilcd.acc[0], 2) + pow(ilcd.acc[2], 2)))* R2D;
			pstOut->roll = atan2(-ilcd.acc[0], ilcd.acc[2])* R2D;

			if (gipro.bDualAntCalibrateOk)
			{
				pstOut->yaw = -(ilcd.heading2 + gipro.dDualAntYawBias);
			}
			else
			{
				pstOut->yaw = -(ilcd.heading2);
			}
			DEG_NEG180_180(pstOut->yaw);
		}
	}
	Rst_speed_pre = pstOut->speed;
	stRaw_tmp.bGPSavail  = false;
	stRaw_tmp.bMEMSavail = false;
	stRaw_tmp.bODOavail  = false;
	stRaw_tmp.bPPSavail  = false;
	return iGilcStatus;
}

gilc_ret_e GilcProcess::GILC_PROCESS_Vehicle_byStrn(char* buff,gilc_result_t* pstOut)
{
	gilc_ret_e ret;
	memset(&stRaw, 0, sizeof(stRaw));
	ret = (gilc_ret_e)GILC_LoadRaw_byStrn(buff,&stRaw);
	if(ret < 0) 
	{
		return ret;
	}
	return GILC_PROCESS_Vehicle(&stRaw,pstOut);
}

int GilcProcess::GILC_Init(gilc_cfg_t* cfgdata)
{
	s_cfgdata = *cfgdata;
	memset(&stRaw,0,sizeof(stRaw));
	memset(Gpospre,0,sizeof(Gpospre));
	
	gipro.Init();
	//gipro.setlever(s_cfgdata.arm_ant);
	kinalign.Init();
	iir_filter.Init();
	ilcd.Init();
	pre_ilcd.Init();

	if(debugfile.bInit)
	{
		debugfile.Close();
	}
	
	sprintf(cGilcInitMsg,"GILC Init ver: %s %s\r\n",GILC_SOFT_VER,GILC_SOFT_DATE);
	sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC NUMX %d, NUMV %d\r\n",NUMX,NUMV);
	
	raw_cnt = 0;
	dbg_cnt = 0;
	rst_cnt = 0;

	Rst_vn_pre = 0;
	Rst_ve_pre = 0;
	Rst_speed_pre = 0;
	static_num = 0; 
	no_static_num = 0;
	iGnssSec_pre = 0;
	GetGPS=false;
	bAlign=false;

	dInstallAttCfg[0] = s_cfgdata.fIns2BodyAngle[0] * D2R;
	dInstallAttCfg[1] = s_cfgdata.fIns2BodyAngle[1] * D2R;
	dInstallAttCfg[2] = s_cfgdata.fIns2BodyAngle[2] * D2R;
	a2mat(dInstallAttCfg, dInstallMat);

	dGnss2OutPointLever[0] = s_cfgdata.fOutPointVector[0];
	dGnss2OutPointLever[1] = s_cfgdata.fOutPointVector[1];
	dGnss2OutPointLever[2] = s_cfgdata.fOutPointVector[2];

	if((int)(s_cfgdata.fWheelDistance[0]*1000))
		InitParam.dWheelTrack = s_cfgdata.fWheelDistance[0];
	else
		InitParam.dWheelTrack = 1.6;
	
	if((int)(s_cfgdata.fWheelDistance[1]*1000))
		InitParam.dWheelBase = s_cfgdata.fWheelDistance[1];
	else
		InitParam.dWheelBase = 2.7;		
	
	if(s_cfgdata.imu_period_ms)
		InitParam.iImuPeriod_ms = s_cfgdata.imu_period_ms;

	InitParam.dInstallErrStdMax_ForInit = 0.15;//0.02
	InitParam.dKfInitP_AccBais = 0.001; /*SCC2X30*/
	InitParam.dKfInitP_GyroBais = 0.01; /*SCC2X30*/
	//InitParam.dKfInitP_AccBais = 0.0001;
	//InitParam.dKfInitP_GyroBais = 0.001;
	InitParam.dKfInitP_InstallErr = 0.05*D2R;
	InitParam.dKfInitP_Lever = 0.01;
	InitParam.dKfInitP_TimeDelay = 0.001;

	InitParam.dKfInitP_OdoKd = 0.05;
	InitParam.dKfInitP_OdoKwh = 0.1;
	InitParam.dKfInitP_OdoBwh = 0.5;
	InitParam.dKfInitP_OdoWh = 0.5;
	InitParam.dKfInitP_DualYaw = 0.1;
	InitParam.dKfInitP_OdoVel = 0.02;
	InitParam.dKfInitP_OdoHeading = 0.1;
	gilc__set_log_level(s_cfgdata.debug_level);
	//gilc__set_log_print_callback(printf);

	switch(s_cfgdata.iWorkMode)
	{
		case GILC_WORK_MODE__DEFAULT:
		case GILC_WORK_MODE__CAR_NORMAL:
		case GILC_WORK_MODE__TRAIN:
		case GILC_WORK_MODE__SHIP:
		case GILC_WORK_MODE__PLANE:
			InitParam.dGnssVerMin_ForInit = 0.3;
			InitParam.bCalibrateUseDualAnt = 0;
			InitParam.iPpsTimeErrMax_ms = 20;
			sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC Work Mode1 %d\r\n",s_cfgdata.iWorkMode);
			break;
		case GILC_WORK_MODE__CAR_SLOW:
		case GILC_WORK_MODE__TRACTOR:
			InitParam.dGnssVerMin_ForInit = 0.5;
			InitParam.bCalibrateUseDualAnt = 1;
			InitParam.iPpsTimeErrMax_ms = 50;
			sprintf(cGilcInitMsg + strlen(cGilcInitMsg), "GILC Work Mode2 %d\r\n",s_cfgdata.iWorkMode);
			break;
		default:
			InitParam.dGnssVerMin_ForInit = 1;
			InitParam.bCalibrateUseDualAnt = 0;
			InitParam.iPpsTimeErrMax_ms = 20;
			gilc_log("Unsupport work mode %d\r\n",s_cfgdata.iWorkMode);
			break;
	}

	sprintf(cGilcInitMsg + strlen(cGilcInitMsg),"GILC cfg.bFilePathCfgUse      %d\r\n",cfgdata->bFilePathCfgUse);
	sprintf(cGilcInitMsg + strlen(cGilcInitMsg),"GILC cfg.bOutFileSaveClose    %d\r\n",cfgdata->bOutFileSaveClose);
	sprintf(cGilcInitMsg + strlen(cGilcInitMsg),"GILC cfg.bTmpFileSaveClose    %d\r\n",cfgdata->bTmpFileSaveClose);
	if(cfgdata->bFilePathCfgUse)
	{
	sprintf(cGilcInitMsg + strlen(cGilcInitMsg),"GILC cfg.debug_outfile_path   %s\r\n",cfgdata->debug_outfile_path);
	sprintf(cGilcInitMsg + strlen(cGilcInitMsg),"GILC cfg.debug_tmpfile_path   %s\r\n",cfgdata->debug_tmpfile_path);
	}
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.bStdCfgUse           %d\r\n",cfgdata->bStdCfgUse);
	if(cfgdata->bStdCfgUse)
	{
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.gyro_std             %lf %lf %lf\r\n",cfgdata->gyro_std[0],cfgdata->gyro_std[1],cfgdata->gyro_std[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.accle_std            %lf %lf %lf\r\n",cfgdata->accle_std[0],cfgdata->accle_std[1],cfgdata->accle_std[2]);
	}
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.bWalkCfgUse          %d\r\n",cfgdata->bWalkCfgUse);
	if(cfgdata->bWalkCfgUse)
	{
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.gyro_walk            %lf %lf %lf\r\n",cfgdata->gyro_walk[0],cfgdata->gyro_walk[1],cfgdata->gyro_walk[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.vel_walk             %lf %lf %lf\r\n",cfgdata->vel_walk[0],cfgdata->vel_walk[1],cfgdata->vel_walk[2]);
	}
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.bRowScaleCfgUse      %d\r\n",cfgdata->bRowScaleCfgUse);
	if(cfgdata->bRowScaleCfgUse)
	{
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.gyro_row             %d %d %d\r\n",cfgdata->gyro_row[0],cfgdata->gyro_row[1],cfgdata->gyro_row[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.acc_row              %d %d %d\r\n",cfgdata->acc_row[0],cfgdata->acc_row[1],cfgdata->acc_row[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.gyro_scale           %lf %lf %lf\r\n",cfgdata->gyro_scale[0],cfgdata->gyro_scale[1],cfgdata->gyro_scale[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.acc_scale            %lf %lf %lf\r\n",cfgdata->acc_scale[0],cfgdata->acc_scale[1],cfgdata->acc_scale[2]);
	}
		
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.bGnssPosStdUse       %d\r\n",cfgdata->bGnssPosStdUse);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.bGnssVelStdUse       %d\r\n",cfgdata->bGnssVelStdUse);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.eGnssVelMode         %d\r\n",cfgdata->eGnssVelMode);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.eGnssPosMode         %d\r\n",cfgdata->eGnssPosMode);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.iOutReferPoint       %d\r\n",cfgdata->iOutReferPoint);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.debug_level          %d\r\n",cfgdata->debug_level);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fIns2BodyVector      %lf %lf %lf\r\n",cfgdata->fIns2BodyVector[0],cfgdata->fIns2BodyVector[1],cfgdata->fIns2BodyVector[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fIns2BodyVectorErr   %lf %lf %lf\r\n",cfgdata->fIns2BodyVectorErr[0],cfgdata->fIns2BodyVectorErr[1],cfgdata->fIns2BodyVectorErr[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fIns2BodyAngle       %lf %lf %lf\r\n",cfgdata->fIns2BodyAngle[0], cfgdata->fIns2BodyAngle[1], cfgdata->fIns2BodyAngle[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fIns2BodyAngleErr    %lf %lf %lf\r\n",cfgdata->fIns2BodyAngleErr[0], cfgdata->fIns2BodyAngleErr[1], cfgdata->fIns2BodyAngleErr[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fIns2GnssVector      %lf %lf %lf\r\n",cfgdata->fIns2GnssVector[0],cfgdata->fIns2GnssVector[1],cfgdata->fIns2GnssVector[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fIns2GnssVectorErr   %lf %lf %lf\r\n",cfgdata->fIns2GnssVectorErr[0],cfgdata->fIns2GnssVectorErr[1],cfgdata->fIns2GnssVectorErr[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fIns2GnssAngle       %lf %lf %lf\r\n",cfgdata->fIns2GnssAngle[0], cfgdata->fIns2GnssAngle[1], cfgdata->fIns2GnssAngle[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fIns2GnssAngleErr    %lf %lf %lf\r\n",cfgdata->fIns2GnssAngleErr[0], cfgdata->fIns2GnssAngleErr[1], cfgdata->fIns2GnssAngleErr[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fOutPointVector      %lf %lf %lf\r\n",cfgdata->fOutPointVector[0], cfgdata->fOutPointVector[1], cfgdata->fOutPointVector[2]);
	sprintf(cGilcInitMsg+strlen(cGilcInitMsg),"GILC cfg.fWheelDistance       %lf %lf\r\n",cfgdata->fWheelDistance[0], cfgdata->fWheelDistance[1]);
	gilc_log(cGilcInitMsg);
	return 0;
}

GilcProcess *s_pstGilc = NULL;
GilcProcess *s_pstGilc_backup = NULL;

int GILC_Cfg_OutReferPoint(int iOutPointType, double dOutPointVector[3])
{
	if (s_pstGilc)
	{
		s_pstGilc->s_cfgdata.iOutReferPoint = iOutPointType;
		s_pstGilc->gipro.ins.SetOutLever(dOutPointVector);

		s_pstGilc->dGnss2OutPointLever[0] = dOutPointVector[0];
		s_pstGilc->dGnss2OutPointLever[1] = dOutPointVector[1];
		s_pstGilc->dGnss2OutPointLever[2] = dOutPointVector[2];

		gilc_log("GILC_Cfg_OutReferPoint %d, %f %f %f\r\n", iOutPointType, dOutPointVector[0], dOutPointVector[1], dOutPointVector[2]);
		return 0;
	}
	return -1;
}

int GILC_Init(gilc_cfg_t* cfgdata)
{
	static GilcProcess gilc;
	s_pstGilc = &gilc;
	return s_pstGilc->GILC_Init(cfgdata);
}

gilc_ret_e GILC_LoadRaw_byStrn(char *buff, gilc_raw_t *pRaw)
{
	if (s_pstGilc)
		return s_pstGilc->GILC_LoadRaw_byStrn(buff, pRaw);
	else
		return GILC_RET__RST_NOTHING;
}

gilc_ret_e GILC_PROCESS_Vehicle(gilc_raw_t* pstRaw,gilc_result_t* pstOut)
{
	if(s_pstGilc)
		return s_pstGilc->GILC_PROCESS_Vehicle(pstRaw,pstOut);
	else
		return GILC_RET__RST_NOTHING;
}

gilc_ret_e GILC_PROCESS_Vehicle_byStrn(char* buff,gilc_result_t* pstOut)
{
	if(s_pstGilc)
		return s_pstGilc->GILC_PROCESS_Vehicle_byStrn(buff,pstOut);
	else
		return GILC_RET__RST_NOTHING;
}

int GILC_Get_EKF_X(ekf_X_t *pstEkfX)
{
	if(s_pstGilc)
	{
		*pstEkfX = s_pstGilc->stEkfX;
	}
	return -1;
}

void GILC_process_backup(void)
{
	static GilcProcess gilc_backup;
	s_pstGilc_backup = &gilc_backup;
	if(s_pstGilc)
		*s_pstGilc_backup = *s_pstGilc;
}

void GILC_process_recovery(void)
{
	if(s_pstGilc&&s_pstGilc_backup)
		*s_pstGilc = *s_pstGilc_backup;
}

int GILC_get_result_inner(gilc_result_inner_t *pstRstInner)
{
	if(s_pstGilc)
	{
		memcpy(pstRstInner,&s_pstGilc->stRstInner,sizeof(gilc_result_inner_t));
		return 0;
	}
	return -1;
}

//int eigen_test()
//{
//	MatrixXd m = MatrixXd::Random(3, 3);          
//	m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;     
//	cout << "m =" << endl << m << endl;
//	VectorXd v(3);     
//	v << 1, 2, 3;       
//	cout << "m * v =" << endl << m * v << endl;
//	//system("pause");
//	return 0;
//}
