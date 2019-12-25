#pragma once
#include "GILC_IMU_Dev.h"
#include "GILC_Boat_lib.h"

/*-------------------------------IMU TYPE-----------------------------*/
#define IMU_DEV_TYPE               IMU_DEV_ADIS16445
#if (IMU_DEV_TYPE == IMU_DEV_ADIS16445)
#include "GILC_IMU_Config_adis16445_nx200.h"
#elif (IMU_DEV_TYPE == IMU_DEV_IMU380)
#include "GILC_IMU_Config_imu380_nx200.h"
#endif

/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_fIns2GnssVector_X    1.0
#define CFG_fIns2GnssVector_Y    0.0
#define CFG_fIns2GnssVector_Z    0.0

/*----------------------------TEST RAW FILE-------------------------*/
#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/NX200/adis16445/20181020/"
#define TEST_RAW_FILE_NAME	   "20181020_090153.txt"
//#define TEST_RAW_FILE_PATH	  "E:/惯性导航/数据处理/NX200/adis16445/20181024/"
//#define TEST_RAW_FILE_NAME	   "20181024_064111.txt"
/*--------------------------------TEST_OUT/TMP_FILE_PATH-----------------------------*/
#define TEST_OUT_FILE_PATH    TEST_RAW_FILE_PATH"OUT/"
#define TEST_TMP_FILE_PATH    "D:/TMP/"

/*--------------------------------TEST_RST_FILE_NAME----------------------------*/
#define TEST_RST_FILE_NAME     "gilc_vehicle_rst.nmea"

/*--------------------------------RAW CFG-----------------------------*/
#define CFG_POS_STD_USE 1
#define CFG_VEL_STD_USE 1
#define CFG_GNSSPOS_MODE GILC_GNSSPOS_MODE__LLA_RAD
#define CFG_GNSSVEL_MODE GILC_GNSSVEL_MODE__ECEF

#define GILC_NMEA_OUT_PERIOD_MS    100     /*dsf90: 输出10Hz*/
