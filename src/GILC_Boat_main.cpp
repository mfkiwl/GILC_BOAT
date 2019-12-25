// GILC_Vehile_test.cpp : 定义控制台应用程序的入口点。
//
#ifdef WIN32 
#include "stdafx.h"
#endif
#ifdef __linux
#include <memory.h>
#endif


#include <time.h>
#ifdef WIN32
#include <windows.h>
#include "configure.h"
#include "configure_calibrate.h"
#else
#  include <sys/time.h>
#endif
#include "hc_type.h"
#include "gilc_out_pack.h"

//#include "GILC_TEST_FileLoad.h"
//#include "GILC_Test_Config_FileLoad_nx200_adis16445.h"
//#include "GILC_Test_Config_FileLoad_P2C_demo_imu381.h"
//#include "GILC_Test_Config_FileLoad_CGI410_imu381.h"
//#include "GILC_Test_Config_FileLoad_P2C_demo_adis16465.h"
//#include "GILC_Test_Config_FileLoad_P2C_demo2_adis16465.h"
//#include "GILC_Test_Config_FileLoad_NX200_demo_imu381.h"
//#include "GILC_Test_Config_FileLoad_CGI300_demo_scc2x30.h"
#include "GD100_IMU381.h"

typedef struct imu_data {
	int    iAccel[3]; /*x,y,z*/
	int    iGyro[3];  /*x,y,z*/
	int    iMag[3];   /*x,y,z*/
	int    iTemp;
	double dAccel[3];                     //accelerometer  X-Y-Z output (m/s2)
	double dGyro[3];                      //gyroscope X-Y-Z output(rad/s)
	double dMag[3];                       //magnetometer
	double dTemp;
}imu_data_t;

#ifndef CFG_WHEEL_TRACK
#if 1/*A级轿车*/
#define CFG_WHEEL_TRACK  1.6f
#define CFG_WHEEL_BASE   2.7f
#elif 0/*EVCAR电动小车*/
#define CFG_WHEEL_TRACK  1.2f
#define CFG_WHEEL_BASE   2.0f
#endif
#endif
/*天线杆臂默认值*/
#ifndef CFG_fIns2GnssVector_X 
#define CFG_fIns2GnssVector_X 0
#endif
#ifndef CFG_fIns2GnssVector_Y 
#define CFG_fIns2GnssVector_Y 0
#endif
#ifndef CFG_fIns2GnssVector_Z 
#define CFG_fIns2GnssVector_Z 0
#endif
#ifndef CFG_fIns2GnssVectorErr_X 
#define CFG_fIns2GnssVectorErr_X 1
#endif
#ifndef CFG_fIns2GnssVectorErr_Y 
#define CFG_fIns2GnssVectorErr_Y 1
#endif
#ifndef CFG_fIns2GnssVectorErr_Z 
#define CFG_fIns2GnssVectorErr_Z 1
#endif
/*天线基线夹角默认值*/
#ifndef CFG_fIns2GnssAngle_X 
#define CFG_fIns2GnssAngle_X 0
#endif
#ifndef CFG_fIns2GnssAngle_Y 
#define CFG_fIns2GnssAngle_Y 0
#endif
#ifndef CFG_fIns2GnssAngle_Z 
#define CFG_fIns2GnssAngle_Z 0
#endif
#ifndef CFG_fIns2GnssAngleErr_X 
#define CFG_fIns2GnssAngleErr_X 5
#endif
#ifndef CFG_fIns2GnssAngleErr_Y 
#define CFG_fIns2GnssAngleErr_Y 5
#endif
#ifndef CFG_fIns2GnssAngleErr_Z 
#define CFG_fIns2GnssAngleErr_Z 5
#endif
/*INS安装误差默认值*/
#ifndef CFG_fIns2BodyAngle_X 
#define CFG_fIns2BodyAngle_X 0
#endif
#ifndef CFG_fIns2BodyAngle_Y 
#define CFG_fIns2BodyAngle_Y 0
#endif
#ifndef CFG_fIns2BodyAngle_Z 
#define CFG_fIns2BodyAngle_Z 0
#endif
#ifndef CFG_fIns2BodyAngleErr_X 
#define CFG_fIns2BodyAngleErr_X 10
#endif
#ifndef CFG_fIns2BodyAngleErr_Y 
#define CFG_fIns2BodyAngleErr_Y 10
#endif
#ifndef CFG_fIns2BodyAngleErr_Z 
#define CFG_fIns2BodyAngleErr_Z 10
#endif

#ifndef CFG_WORK_MODE 
#define CFG_WORK_MODE GILC_WORK_MODE__CAR_NORMAL
#endif

HC_INT32 s_iGilcNmeaOutPeriod_ms  = 200;     
HC_INT32 s_iGilcCfgUpdatePeriod_ms  = 60000;
static HC_UINT64 s_iOutCasco_time = 0;
gnss_data_t stGnssData = { 0 };
static HC_INT32 s_iCalibrateCfgUsed = 0;

#ifdef WIN32
int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = (long)clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

void imu_test(imu_data_t stImuData, double dt, double	imu_gyro_deg[3])
{
	int i;
	static	int bias_cnt = 0;
	static	int init_flag = 0;
	static  double	imu_acc_sum[3] = { 0.0 };
	static  double	imu_gyro_sum[3] = { 0.0 };
	//static  double	imu_acc_bias[3] = {0.0};
	static  double	imu_gyro_bias[3] = { 0.0 };

	if (!init_flag)
	{
		if (bias_cnt < 100 * 10)
		{
			bias_cnt++;
			for (i = 0; i<3; i++)
			{
				//imu_acc_sum[i] +=  stImuData.dAccel[i];
				imu_gyro_sum[i] += stImuData.dGyro[i];
			}
		}
		else
		{
			init_flag = 1;
		}
		for (i = 0; i<3; i++)
		{
			//imu_acc_bias[i] =  imu_acc_sum[i]/bias_cnt;
			imu_gyro_bias[i] = imu_gyro_sum[i] / bias_cnt;
		}
		printf("Get GYRO BIAS: X,%3.3f	Y,%3.3f  Z,%3.3f\r\n", imu_gyro_bias[0], imu_gyro_bias[1], imu_gyro_bias[2]);
	}
	else
	{
		for (i = 0; i<3; i++)
		{
			if (dt < 0.2)
				imu_gyro_deg[i] += (stImuData.dGyro[i] - imu_gyro_bias[i])*dt;

			if (imu_gyro_deg[i]>360)
				imu_gyro_deg[i] -= 360;
			if (imu_gyro_deg[i]<0)
				imu_gyro_deg[i] += 360;
		}
		printf("GYRO DEG: X,%4.3f	Y,%4.3f   Z,%4.3f  ", imu_gyro_deg[0], imu_gyro_deg[1], imu_gyro_deg[2]);
	}
}

FILE *fd_rst = NULL;
HC_VOID gilc_save__rst_data_save( HC_UINT8 *data, HC_UINT32 len )
{
	if (fd_rst)
	{
		//gilc_log_hex("data       : ", 50, data, len);
		//printf("fwrite len %d\r\n", len);
		len = fwrite(data,len,1,fd_rst);
		//fflush(fd_rst);
	}
}

HC_VOID gilc_process__output_casco(gilc_raw_t* pstRaw, gilc_result_t *pstOut)
{
	HC_UINT8 cBuffer[1024] = { 0 };
	HC_UINT16  sSaveLen = 0;
	HC_UINT64 weektime_new = (HC_UINT64)(pstRaw->gnssdata.week * 7 * 24 * 3600 * 1000.0 + pstRaw->gnssdata.second*1000.0);
	if (weektime_new > s_iOutCasco_time)
	{
		sSaveLen = GILC_IOOut_CreatCascoPack(cBuffer,pstRaw,pstOut);
		gilc_save__rst_data_save(cBuffer, sSaveLen);
		//Write2FIFO(cBuffer, sSaveLen, COM2);
		s_iOutCasco_time = weektime_new;
	}
	else
	{
		printf("ERR: Gnss remove by code, new time %.3f, last output time %.3f------------\r\n", pstRaw->gnssdata.second, (HC_DOUBLE)(s_iOutCasco_time % (7 * 24 * 3600 * 1000)) / 1000);
	}
}

HC_VOID gilc_process__rst_prc(gilc_ret_e eGilcRet, gilc_raw_t *pstRaw, gilc_result_t *pstOut)
{
	static HC_UINT64 gilc_out_com_ctl_times = 0;
	static HC_UINT64 gilc_out_ctl_times = 0;
	static HC_UINT64 gilc_cfg_ctl_times = 0;

	HC_UINT8 cBuffer[1024] = { 0 };
	HC_UINT16  sSaveLen = 0;
#if 0
	//printf("gilc ret %d\r\n",eGilcRet);
	if (eGilcRet == GILC_RET__RST_INITING || eGilcRet == GILC_RET__RST_RESOLVING)
	{
		imu_data_t stImuData;
		stImuData.dAccel[0] = pstRaw->memsdate.accel[0];
		stImuData.dAccel[1] = pstRaw->memsdate.accel[1];
		stImuData.dAccel[2] = pstRaw->memsdate.accel[2];
		stImuData.dGyro[0] = pstRaw->memsdate.gyro[0];
		stImuData.dGyro[1] = pstRaw->memsdate.gyro[1];
		stImuData.dGyro[2] = pstRaw->memsdate.gyro[2];

		struct timeval tv_last = { 0 }, tv_new = {0};
		gettimeofday(&tv_new, NULL);
		if (tv_last.tv_sec == 0)
			tv_last = tv_new;
		double dt = (tv_new.tv_sec - tv_last.tv_sec) + (tv_new.tv_usec - tv_last.tv_usec)*1e-6;
		tv_last = tv_new;

		double	imu_gyro_deg[3] = { 0 };
		imu_test(stImuData, dt, imu_gyro_deg);
		pstOut->pitch = imu_gyro_deg[0];
		pstOut->roll = imu_gyro_deg[1];
		pstOut->yaw = imu_gyro_deg[2];
		pstOut->heading = imu_gyro_deg[2];
		pstOut->bHeadingOk = 1;
	}
#endif

	if (eGilcRet > 0 && pstOut->week != 0)
	{
		pstOut->second += 0.00000005;
		HC_UINT64 weektime_new = (HC_UINT64)(pstOut->week * 7 * 24 * 3600 * 1000.0 + pstOut->second*1000.0);
		HC_INT32 iTimeSec = (HC_INT32)round(pstOut->second * 1000);
		//if (gilc_out_ctl_times == 0 && (iTimeSec%1000) == 0)
			if (gilc_out_ctl_times == 0)
		{
			gilc_out_ctl_times = weektime_new - (weektime_new % 200);
			printf("gilc_out_ctl_times init: %d,%.6f;%llu,%llu\r\n", pstOut->week, pstOut->second, weektime_new, gilc_out_ctl_times);
		}
		if (gilc_out_ctl_times != 0 && weektime_new >= gilc_out_ctl_times)
		{
			gilc_out_ctl_times += GILC_NMEA_OUT_PERIOD_MS;
			sSaveLen = GILC_IOOut_CreatNmeaPack(cBuffer, pstRaw, pstOut, GILC_NMEA_OUT_FLAG);
			if (sSaveLen)
				gilc_save__rst_data_save(cBuffer, sSaveLen);
			//NMEA_CHC_recv((char*)cBuffer,sSaveLen);
			//gilc_log_hex("nmea: ",30,cBuffer, sSaveLen);
			
			//printf("POS: %8.3f   %d ;",pstOut->second,pstOut->week);
			//printf("POS: %4.3f	 %4.3f  %4.3f ;",pstOut->lla[0],pstOut->lla[1],pstOut->lla[2]);
			//printf("STD: %4.3f	 %4.3f  %4.3f ;",pstOut->std_lla[0],pstOut->std_lla[1],pstOut->std_lla[2]);
			//printf("VEL: %4.3f	 %4.3f  %4.3f ;",pstOut->vel_enu[0],pstOut->vel_enu[1],pstOut->vel_enu[2]);
			//printf("STD: %4.3f	 %4.3f  %4.3f ;",pstOut->std_vel[0],pstOut->std_vel[1],pstOut->std_vel[2]);
			//printf("PRY: %4.3f	 %4.3f  %4.3f ;",pstOut->pitch,pstOut->roll,pstOut->yaw);
			//printf("STD: %4.3f	 %4.3f  %4.3f ;",pstOut->std_pry[0],pstOut->std_pry[1],pstOut->std_pry[2]);
			//printf("SPEED: %4.3f; HEADING  %4.3f;",pstOut->speed,pstOut->heading);
			//printf("FLAG: %d %d %d %d %d\r\n",pstOut->bPPSSync,pstOut->bHeadingOk,pstOut->iSensorFlag,pstOut->car_status,pstOut->gilc_status);
			//printf("\r\n");
		}
		/*
		if (gilc_out_com_ctl_times == 0)
		{
			gilc_out_com_ctl_times = weektime_new - (weektime_new%s_iGilcNmeaOutPeriod_ms);
			printf("gilc_out_com_ctl_times init: %d,%.6f;%llu,%llu\r\n", pstOut->week, pstOut->second, weektime_new, gilc_out_ctl_times);
		}
		if (weektime_new >= gilc_out_com_ctl_times)
		{
			gilc_out_com_ctl_times += s_iGilcNmeaOutPeriod_ms;
			sSaveLen = GILC_IOOut_CreatNmeaPack(cBuffer, pstRaw, pstOut,0x0007);
			if (sSaveLen && s_iOutMsg_flag)
				Write2FIFO(cBuffer, sSaveLen, COM2);
		}
		*/
#ifdef WIN32
//#if GILC_CALIBRATE_SAVE_USED
		if (gilc_cfg_ctl_times == 0)
		{
			gilc_cfg_ctl_times = weektime_new - (weektime_new%s_iGilcCfgUpdatePeriod_ms);
			printf("gilc_cfg_ctl_times init: %d,%.6f;%llu,%llu\r\n", pstOut->week, pstOut->second, weektime_new, gilc_cfg_ctl_times);
		}

		//if (weektime_new >= gilc_cfg_ctl_times)
		{
			HC_INT8 cFilePath[128] = { 0 };
			sprintf((char *)cFilePath, "%sOut/gilc_calibrate_cfg", TEST_RAW_FILE_PATH);

			config_calibrate_t stCfgData;
			memset(&stCfgData, 0, sizeof(stCfgData));

			stCfgData.time[0] = pstOut->week;
			stCfgData.time[1] = pstOut->second;
			GILC_Get_EKF_X(&stCfgData.stEkfX);
			stCfgData.iCrc = hc_crc32(&stCfgData,(HC_INT32)&stCfgData.iCrc-(HC_INT32)&stCfgData);

			gilc_cfg_ctl_times += s_iGilcCfgUpdatePeriod_ms;
			if (!s_iCalibrateCfgUsed && eGilcRet >= GILC_RET__RST_STABLE)
			{
				static int update_flag = 0;
				if(!update_flag) /*GILC_RET__RST_STABLE后1分钟内，更新配置*/
				{
					update_flag++;
					//config_calibrate__update(&stCfgData);/*dsf90:imx6ul耗时约50ms，影响imu数据采集*/ 
					gilc__calibrate_cfg_save2(cFilePath,&stCfgData);
				}
			}
		}
#endif
	}
	else if (eGilcRet < 0 && eGilcRet != -3)
	{
		printf("gilc ret %d, time %d ,%.3f\r\n", eGilcRet, pstOut->week, pstOut->second);
	}
#if 0		
	if (eGilcRet <= 0)
	{
		printf("Gilc ret %d, string: %s\r\n", eGilcRet, (HC_INT8 *)dat);
	}
	else
	{
		static int cnt = 0;
		if (++cnt == 100)
		{
			cnt = 0;
			logi("Gilc ret %d, GNSS+IMU:%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
				eGilcRet,
				pstOut->second, pstOut->week,
				pstOut->lla_imu[0], pstOut->lla_imu[1], pstOut->lla_imu[2],
				pstOut->lla_ant[0], pstOut->lla_ant[1], pstOut->lla_ant[2],
				pstOut->ver_enu[0], pstOut->ver_enu[1], pstOut->ver_enu[2],
				pstOut->pitch*R2D, pstOut->roll*R2D, pstOut->yaw*R2D,
				pstOut->speed, pstOut->heading,
				pstOut->car_status, pstOut->gilc_status);
		}
		}
#endif
}


int gilc_process_main(char *cRawFilePath,char *cRawFileName,char *cOutFilePath,char *cTmpFilePath)
{
	int iRet = 0;
	gilc_ret_e eGilc_strRet = GILC_RET__RST_NOTHING;
	gilc_ret_e eGilcRet = GILC_RET__RST_NOTHING;
	gilc_raw_t stRaw = { 0 }, stRawTmp = { 0 };
	static gilc_result_t stOut = { 0 };
	gilc_cfg_t stCfg = { 0 };
	char buff[MAXLEN] = { 0 };
	FILE *fd = NULL;

#if 0
	FILE *fp_test = NULL;
	fp_test = fopen("G:/GD100项目/测试数据/9.5外场测试/data2/heading.txt", "wt");
#endif

	char lcfile[1024] = {0};
	char lcfile_outpath[1024] = {0};
	char lcfile_tmppath[1024] = {0};
	unsigned long long gilc_out_ctl_times = 0;

	sprintf(lcfile,"%s%s",cRawFilePath,cRawFileName);
	sprintf(lcfile_outpath,"%s",cOutFilePath);
	sprintf(lcfile_tmppath,"%s",cTmpFilePath);

#if 0
	HC_BOOL s_bGilcProcessRt_flag = 0;
	HC_BOOL s_bGilcProcessRtStrn_flag = 0;
	HC_INT32 s_iGilcNmeaOutPeriod_ms = 0;
	HC_INT32 s_iGilcImuReadPeriod_ms = 0;
	HC_INT32 s_iGilcImuBw_hz = 10;

	HC_INT32  install_mode[3] = { -2,1,3 };/*demo2*/
	HC_DOUBLE lever_init[3] = { 0.0 };
	HC_DOUBLE wheelbase = 0.0, wheeltrack = 0.0;
	int ret;

	if (init_config()<0)
	{
		creat_config();
		config_set_imu_read_freq(100);
		config_set_imu_bw_freq(40);
		config_set_imu_installtype(install_mode);
		config_set_gilc_out_period(20);
		config_set_gilc_process_rt(1);
		config_set_gilc_process_strn(0);
		config_set_gilc_lever(lever_init);
		config_set_gilc_wheeltrack(1.6);
		config_set_gilc_wheelbase(2.7);
		save_config();
	}

	s_iGilcImuBw_hz = config_get_imu_bw_freq();
	s_iGilcImuReadPeriod_ms = 1000 / config_get_imu_read_freq();
	s_iGilcNmeaOutPeriod_ms = config_get_gilc_out_period();
	s_bGilcProcessRt_flag = config_get_gilc_process_rt();
	s_bGilcProcessRtStrn_flag = config_get_gilc_process_strn();

	config_get_imu_installtype(install_mode);
	config_get_gilc_lever(lever_init);

	wheeltrack = config_get_gilc_wheeltrack();
	wheelbase = config_get_gilc_wheelbase();

	printf("config_get_imu_install_xyz = %d %d %d\r\n", install_mode[0], install_mode[1], install_mode[2]);
	printf("config_get_imu_bw_freq = %d\r\n", s_iGilcImuBw_hz);
	printf("config_get_imu_period = %d\r\n", s_iGilcImuReadPeriod_ms);
	printf("config_get_gilc_out_period = %d\r\n", s_iGilcNmeaOutPeriod_ms);
	printf("config_get_gilc_process_rt = %d\r\n", s_bGilcProcessRt_flag);
	printf("config_get_gilc_process_strn = %d\r\n", s_bGilcProcessRtStrn_flag);
	printf("config_get_gilc_lever = %.3f %.3f %.3f\r\n", lever_init[0], lever_init[1], lever_init[2]);
	printf("config_get_gilc_wheeltrack/wheelbase = %.3f %.3f\r\n", wheeltrack, wheelbase);
	return 0;
#endif

//#if 0
#ifdef WIN32
	config_calibrate_t stCfgData;
	HC_INT8 cFilePath[128] = { 0 };
	memset(&stCfgData, 0, sizeof(stCfgData));
	sprintf((char *)cFilePath, "%sOut/gilc_calibrate_cfg", TEST_RAW_FILE_PATH);
	//iRet = config_calibrate__read(&stCfgData);
	iRet = gilc__calibrate_cfg_read2(cFilePath,&stCfgData);
	if(iRet == HC_OK)
	{
		if(stCfgData.iCrc == hc_crc32(&stCfgData,(HC_INT32)&stCfgData.iCrc-(HC_INT32)&stCfgData))
		{
			stCfg.stEkfX_Init = stCfgData.stEkfX;
			stCfg.bEkfXUse = true;
			s_iCalibrateCfgUsed = 1;
		}
		else
		{
			printf("config_calibrate__read crc32 error \r\n");
		}
	}
#endif

#if (!GILC_DEBUG_PROCESS_TIME)
	strncpy(stCfg.debug_outfile_path, lcfile_outpath, sizeof(stCfg.debug_outfile_path));
	strncpy(stCfg.debug_tmpfile_path, lcfile_tmppath, sizeof(stCfg.debug_tmpfile_path));
	stCfg.debug_level = 1;
	stCfg.bFilePathCfgUse = true;
	//stCfg.bOutFileSaveClose = true;
	//stCfg.bTmpFileSaveClose = true;
#endif

#if 1
#if 1
	stCfg.gyro_std[0] = GYRO_STD_X;
	stCfg.gyro_std[1] = GYRO_STD_Y;
	stCfg.gyro_std[2] = GYRO_STD_Z;
	stCfg.accle_std[0] = ACC_STD_X;
	stCfg.accle_std[1] = ACC_STD_Y;
	stCfg.accle_std[2] = ACC_STD_Z;
	stCfg.bStdCfgUse = true;

	stCfg.gyro_walk[0] = stCfg.gyro_walk[1] = stCfg.gyro_walk[2] = GYRO_WALK;
	stCfg.vel_walk[0] = stCfg.vel_walk[1] = stCfg.vel_walk[2] = VEL_WALK;
	stCfg.bWalkCfgUse = true;
#endif

	stCfg.gyro_row[0] = CFG_GYRO_X_ROW;
	stCfg.gyro_row[1] = CFG_GYRO_Y_ROW;
	stCfg.gyro_row[2] = CFG_GYRO_Z_ROW;
	stCfg.acc_row[0] = CFG_ACC_X_ROW;
	stCfg.acc_row[1] = CFG_ACC_Y_ROW;
	stCfg.acc_row[2] = CFG_ACC_Z_ROW;
	stCfg.gyro_scale[0] = CFG_GYRO_X_SCALE;
	stCfg.gyro_scale[1] = CFG_GYRO_Y_SCALE;
	stCfg.gyro_scale[2] = CFG_GYRO_Z_SCALE;
	stCfg.acc_scale[0] = CFG_ACC_X_SCALE;
	stCfg.acc_scale[1] = CFG_ACC_Y_SCALE;
	stCfg.acc_scale[2] = CFG_ACC_Z_SCALE;
	stCfg.bRowScaleCfgUse = true;

	stCfg.bGnssPosStdUse = CFG_POS_STD_USE;
	stCfg.bGnssVelStdUse = CFG_VEL_STD_USE;
	stCfg.eGnssVelMode = CFG_GNSSVEL_MODE;
	stCfg.eGnssPosMode = CFG_GNSSPOS_MODE;
	stCfg.fWheelDistance[0] = CFG_WHEEL_TRACK;
	stCfg.fWheelDistance[1] = CFG_WHEEL_BASE;
	stCfg.iWorkMode         = CFG_WORK_MODE;

	/*天线杆臂*/
	stCfg.fIns2GnssVector[0] = CFG_fIns2GnssVector_X;
	stCfg.fIns2GnssVector[1] = CFG_fIns2GnssVector_Y;
	stCfg.fIns2GnssVector[2] = CFG_fIns2GnssVector_Z;
	stCfg.fIns2GnssVectorErr[0] = CFG_fIns2GnssVectorErr_X;
	stCfg.fIns2GnssVectorErr[1] = CFG_fIns2GnssVectorErr_Y;
	stCfg.fIns2GnssVectorErr[2] = CFG_fIns2GnssVectorErr_Z;
	/*天线基线夹角*/
	stCfg.fIns2GnssAngle[0] = CFG_fIns2GnssAngle_X;
	stCfg.fIns2GnssAngle[1] = CFG_fIns2GnssAngle_Y;
	stCfg.fIns2GnssAngle[2] = CFG_fIns2GnssAngle_Z;
	stCfg.fIns2GnssAngleErr[0] = CFG_fIns2GnssAngleErr_X;
	stCfg.fIns2GnssAngleErr[1] = CFG_fIns2GnssAngleErr_Y;
	stCfg.fIns2GnssAngleErr[2] = CFG_fIns2GnssAngleErr_Z;
	/*INS安装夹角*/
	stCfg.fIns2BodyAngle[0] = CFG_fIns2BodyAngle_X;
	stCfg.fIns2BodyAngle[1] = CFG_fIns2BodyAngle_Y;
	stCfg.fIns2BodyAngle[2] = CFG_fIns2BodyAngle_Z;
	stCfg.fIns2BodyAngleErr[0] = CFG_fIns2BodyAngleErr_X;
	stCfg.fIns2BodyAngleErr[1] = CFG_fIns2BodyAngleErr_Y;
	stCfg.fIns2BodyAngleErr[2] = CFG_fIns2BodyAngleErr_Z;
	/*输出目标点杆臂*/
#ifdef CFG_fGnss2Outpoint_X
	stCfg.iOutReferPoint = GILC_OUTREFER_POINT__INPUT;
	stCfg.fOutPointVector[0] = CFG_fGnss2Outpoint_X;
	stCfg.fOutPointVector[1] = CFG_fGnss2Outpoint_Y;
	stCfg.fOutPointVector[2] = CFG_fGnss2Outpoint_Z;
#else
	stCfg.iOutReferPoint = GILC_OUTREFER_POINT__GNSS;
#endif

#endif
	GILC_Init(&stCfg);

	//double dOutPointVector[3] = {CFG_fGnss2Outpoint_X,CFG_fGnss2Outpoint_Y,CFG_fGnss2Outpoint_Z };
	//GILC_Cfg_OutReferPoint(GILC_OUTREFER_POINT__INPUT, dOutPointVector);

	fd = fopen(lcfile, "rt");
	if (!fd)
	{
		printf("open file err! %s\r\n", lcfile);
#ifdef WIN32
		system("pause");
#endif
		return 0;
	}

	static unsigned long long llu_once = 0, llu_ave = 0, llu_total = 0, llu_cnt = 0;
	//struct timeval tv, tv_last;
	while (1)
	{
#ifdef GILC_MEM_USE
		//GILC_MEM_INFO  mon_p;
		//gilc_getMemInfo(&mon_p);
#endif

#if 0
		/*dsf90:debug 重新初始化、文件存储测试*/
		static long lDebug_cnt = 0;
		if (lDebug_cnt++ == 80000)
		{
			GILC_Init(&stCfg);
		}
#endif		
		if (feof(fd))
		{
			printf("file read over!\n");
			if (fd)
				fclose(fd);
			if (fd_rst)
				fclose(fd_rst);
			break;
		}

		fgets(buff, MAXLEN, fd);
		if (strlen(buff)<18)
		{
			printf("file read error: %s\n", buff);
			continue;
		}

#if 1		
		/*dsf90:debug 接口测试*/
		//memset(&stRaw, 0, sizeof(stRaw));
		eGilc_strRet = (gilc_ret_e)GILC_LoadRaw_byStrn(buff, &stRawTmp);
		if (eGilc_strRet == -1)
		{
			printf("GILC_Load param err!\r\n");
			continue;
		}
		else if (eGilc_strRet == -2)
		{
			printf("GILC_Load fail, unknow strning: %s\n", buff);
			continue;
		}

		if (stRawTmp.bODOavail)
		{
			stRaw.bODOavail = stRawTmp.bODOavail;
			stRaw.ododata = stRawTmp.ododata;
		}

		if (stRawTmp.bGPSavail)
		{
			stRaw.bGPSavail = stRawTmp.bGPSavail;
			stRaw.gnssdata = stRawTmp.gnssdata;
			stRaw.gnss_delay_ms = stRawTmp.gnss_delay_ms;
		}

		if (stRawTmp.bMEMSavail)
		{
			stRaw.bMEMSavail = stRawTmp.bMEMSavail;
			stRaw.bPPSavail = stRawTmp.bPPSavail;
			stRaw.imutimetarget = stRawTmp.imutimetarget;
			stRaw.memsdate = stRawTmp.memsdate;
		}

		static double start_time = 0.0;
		if ((stRaw.gnssdata.second > start_time + 1 * 60 && stRaw.gnssdata.second <  start_time + 2 * 60))
		{
			//stRaw.bGPSavail = false;
			//stRaw.bODOavail = false;
		}
		if ((stRaw.gnssdata.second >185909&& stRaw.gnssdata.second <  185935))
		{
			stRaw.bGPSavail = false;
			//stRaw.bODOavail = false;
		}
		if ((stRaw.gnssdata.second > 466647 ))
		{
			//stRaw.bGPSavail = false;
			//stRaw.bODOavail = false;
		}
		
		if (stRaw.gnssdata.second <= 111627)
		{
			//stRaw.bGPSavail = false;
			//continue;
			//stRaw.bODOavail = false;
		} 
		
		memset(&stOut, 0, sizeof(stOut));
		eGilcRet = GILC_PROCESS_Vehicle(&stRaw, &stOut);
		//if (stRaw.imutimetarget > 0)
		//{
		//	fprintf(fp_test, "%f,%f,%f,%f,%f,%d,%d\n",
		//		stRaw.imutimetarget, stOut.yaw, stOut.heading,
		//		stRaw.gnssdata.heading2,stOut.speed,
		//		stRaw.bGPSavail, stRaw.bMEMSavail);
		//}
		if (!start_time && stRaw.imutimetarget && eGilcRet >= GILC_RET__RST_STABLE)
			//if (!start_time && stRaw.imutimetarget && eGilcRet >= GILC_RET__RST_RESOLVING)
				start_time = stRaw.imutimetarget;




#if 0 //GILC_Cfg_OutReferPoint()接口测试
		if ((stRaw.gnssdata.second > start_time + 2* 60 && stRaw.gnssdata.second <  start_time + 4 * 60))
		{
			double dOutPointVector[3] = {-1,0,0};
			GILC_Cfg_OutReferPoint(3, dOutPointVector);
		}
		if ((stRaw.gnssdata.second > start_time + 4 * 60 && stRaw.gnssdata.second <  start_time + 6 * 60))
		{
			double dOutPointVector[3] = { 1,0,0 };
			GILC_Cfg_OutReferPoint(3, dOutPointVector);
		}
		if ((stRaw.gnssdata.second > start_time + 6 * 60 && stRaw.gnssdata.second <  start_time + 8 * 60))
		{
			double dOutPointVector[3] = { 0,0,0 };
			GILC_Cfg_OutReferPoint(3, dOutPointVector);
		}
#endif

#else
		//gettimeofday(&tv_last, NULL);

		/*dsf90:debug 接口测试*/
		memset(&stOut, 0, sizeof(stOut));
		eGilcRet = GILC_PROCESS_Vehicle_byStrn(buff, &stOut);

		//gettimeofday(&tv, NULL);		//get system time here to get ms info, tv.tv_usec means ms
		//llu_once = (tv.tv_sec - tv_last.tv_sec) * 1000000u + tv.tv_usec - tv_last.tv_usec;
		//llu_cnt++; llu_total += llu_once; llu_ave = llu_total / llu_cnt;

#if GILC_DEBUG_PROCESS_TIME
		printf("---debug process time---:count,%12llu; onec,%12llu; total,%12llu; ave,%12llu\r\n",
			llu_cnt, llu_once, llu_total, llu_ave);
#endif
#endif
		gilc_process__rst_prc(eGilcRet, &stRaw,&stOut);
		stRaw.bGPSavail = 0;
		stRaw.bMEMSavail = 0;
		stRaw.bODOavail = 0;
		stRaw.bPPSavail = 0;

#ifdef OUT_CASCO
		HC_UINT64 weektime_new = (HC_UINT64)(stOut.week * 7 * 24 * 3600 * 1000.0 + stOut.second*1000.0);
		if (weektime_new > (s_iOutCasco_time + 200 + 120)) /*GNSS延迟120ms,默认数据丢失*/
		{
			HC_UINT64 gnsstime_new = s_iOutCasco_time + 200;
			stRaw.gnssdata = stGnssData;
			stRaw.bGPSavail = 1;
			stRaw.gnss_delay_ms = 120;
			stRaw.gnssdata.second = ((HC_DOUBLE)(gnsstime_new % (7 * 24 * 3600 * 1000))) / 1000.0;
			stRaw.gnssdata.week = (HC_INT32)(gnsstime_new / (7 * 24 * 3600 * 1000));
			//printf("--- %llu  %llu  %llu----", weektime_new, s_iOutCasco_time, gnsstime_new);
			//printf("imu:%d,%lf  gnss:%d,%lf\r\n", stOut.week, stOut.second, stRaw.gnssdata.week, stRaw.gnssdata.second);
			//printf("GNSS delay/lost, output gilc, new time %.3f, last time %.3f------------\r\n", stRaw.gnssdata.second, (HC_DOUBLE)(s_iOutCasco_time % (7 * 24 * 3600 * 1000)) / 1000);
			gilc_process__output_casco(&stRaw, &stOut);
		}
#endif

		if (!fd_rst && stOut.week)
		{
			char lcfile_rst[128] = {0};
			sprintf(lcfile_rst, "%s%d_%.2f_ReferPoint%d_rst.nmea", TEST_OUT_FILE_PATH, stOut.week,stOut.second, stCfg.iOutReferPoint);
			fd_rst = fopen(lcfile_rst, "wb");
			if (!fd_rst)
				printf("open file err! %s\r\n", lcfile_rst);
		}
	}
	printf("---debug process time---:count,%12llu; onec,%12llu us; total,%12llu s; ave,%12llu us\r\n",
		llu_cnt, llu_once, (llu_total / 1000000u), llu_ave);

	return 0;
}

		
int process_while()
{
	/*
	char cRawFileName[][128] = {
			"012105_raw.txt",
			"061630_raw.txt",
			"100327_raw.txt",
			"115249_raw.txt",
			"121157_raw.txt",
			"122313_raw.txt",
			"125310_raw.txt",
			"130359_raw.txt",
			"134859_raw.txt"
		};*/
	char cRawFileName[][128] = {
		"084808_raw.txt",
		"094844_raw.txt",
		"121359_raw.txt",
		"133237_raw.txt",
		"134649_raw.txt"
	};

	char cOutFilePath[1024] = {0};
	char cTmpFilePath[1024] = {0};
	int i = 0,j=0;
	int num = ARRAY_SIZE(cRawFileName);

	num = 1;
	
#ifdef WIN32
	sprintf(cTmpFilePath,"%s","D:/TMP/");
#else
	sprintf(cTmpFilePath,"%s","/mnt/gilc-data/TMP/");
#endif	
	for(j=0;j<1;j++)
	{
		sprintf(cOutFilePath,"%sOUT_%d/",TEST_RAW_FILE_PATH,j);
		for(i= 0;i<num;i++)
			gilc_process_main(TEST_RAW_FILE_PATH,cRawFileName[i], cOutFilePath, cTmpFilePath);
	}
	return 0;
}

//int eigen_main()
//{
//	MatrixXd m = MatrixXd::Random(3, 3);              //随机生成3*3的double型矩阵
//	m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;      //MatrixXd::Constant(3,3,1.2)表示生成3*3的double型矩阵，该矩阵所有元素均为1.2
//	cout << "m =" << endl << m << endl;
//	VectorXd v(3);        // 定义v为3*1的double型向量
//	v << 1, 2, 3;         // 向量赋值
//	cout << "m * v =" << endl << m * v << endl;
//	system("pause");
//	return 0;
//}

int main()
{
	//return eigen_main();

#ifdef GILC_MEM_USE
	char * gilc_mem = (char *)malloc(256*1024);
	gilc_MemPoolInit(gilc_mem, 256 * 1024);
#endif

	char cOutFilePath[1024] = { 0 };
	char cTmpFilePath[1024] = { 0 };
	//process_while();
	sprintf(cOutFilePath, "%sOUT/", TEST_RAW_FILE_PATH);
	
#ifdef WIN32
	sprintf(cTmpFilePath,"%s","D:/TMP/");
#else
	sprintf(cTmpFilePath,"%s","/mnt/gilc-data/TMP/");
#endif
	gilc_process_main(TEST_RAW_FILE_PATH, TEST_RAW_FILE_NAME, cOutFilePath, "D:/TMP/");

#ifdef GILC_MEM_USE
	free(gilc_mem);
#endif
#ifdef WIN32
	system("pause");
#endif
}

