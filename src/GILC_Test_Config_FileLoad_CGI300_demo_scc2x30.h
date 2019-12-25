#pragma once
#include "GILC_IMU_Dev.h"
#include "GILC_Boat_lib.h"

/*-------------------------------IMU TYPE-----------------------------*/
#include "GILC_IMU_Config_scc2x30.h"

/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_fIns2GnssVector_X    0
#define CFG_fIns2GnssVector_Y    0
#define CFG_fIns2GnssVector_Z    0

/*----------------------------TEST RAW FILE-------------------------*/
#ifdef WIN32
//#define WORK_PATH  "E:/惯性导航/数据处理/"
#define WORK_PATH  "D:/数据处理/"
#else
#define WORK_PATH  "/mnt/gilc-data/"
#endif


//#define TEST_RAW_FILE_PATH	 WORK_PATH"导航模块-村田/20190610/"   
//#define TEST_RAW_FILE_NAME	 "1.dat"

#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-300/20190702/"   
#define TEST_RAW_FILE_NAME	 "101528_raw.txt"

/*--------------------------------TEST_OUT/TMP_FILE_PATH-----------------------------*/
#define TEST_OUT_FILE_PATH    TEST_RAW_FILE_PATH"OUT/"
#define TEST_TMP_FILE_PATH    "D:/TMP/"

/*--------------------------------TEST_RST_FILE_NAME----------------------------*/
#define TEST_RST_FILE_NAME     "gilc_vehicle_rst.nmea"

/*--------------------------------RAW CFG-----------------------------*/
#define CFG_POS_STD_USE 1
#define CFG_VEL_STD_USE 1
#define CFG_GNSSPOS_MODE GILC_GNSSPOS_MODE__LLA_DEG
#define CFG_GNSSVEL_MODE GILC_GNSSVEL_MODE__ECEF

#define GILC_NMEA_OUT_PERIOD_MS    100     
#define GILC_NMEA_OUT_FLAG         0x0007    
