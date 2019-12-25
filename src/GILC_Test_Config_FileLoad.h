#pragma once
#include "GILC_IMU_Dev.h"
#include "GILC_Boat_lib.h"

/*-------------------------------IMU TYPE-----------------------------*/
#define IMU_DEV_TYPE               IMU_DEV_IMU381
#if (IMU_DEV_TYPE == IMU_DEV_ADIS16445)
#include "GILC_IMU_Config_adis16445.h"
#elif (IMU_DEV_TYPE == IMU_DEV_ADIS16465)
#include "GILC_IMU_Config_adis16465.h"
#elif (IMU_DEV_TYPE == IMU_DEV_IMU380)
#include "GILC_IMU_Config_imu380.h"
#elif (IMU_DEV_TYPE == IMU_DEV_IMU381)
#include "GILC_IMU_Config_imu381.h"
#endif

/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_fIns2GnssVector_X    0.0
#define CFG_fIns2GnssVector_Y    0.0
#define CFG_fIns2GnssVector_Z    0.0

/*----------------------------TEST RAW FILE-------------------------*/
//#define TEST_RAW_FILE_PATH	  "E:/���Ե���/���ݴ���/20180830_P2_ADIS16465/"
//#define TEST_RAW_FILE_NAME	   "070320_raw.txt"
//#define TEST_RAW_FILE_PATH	  "E:/���Ե���/���ݴ���/P2C/imu381/20181017/"
//#define TEST_RAW_FILE_NAME	   "011610_raw.txt"
//#define TEST_RAW_FILE_PATH	  "E:/���Ե���/���ݴ���/P2C/imu381/20181019/"
//#define TEST_RAW_FILE_NAME	   "060137_raw.txt"/*԰������*/
//#define TEST_RAW_FILE_NAME	   "063038_raw.txt"/*ӭ����·-����-����-԰������*/
//#define TEST_RAW_FILE_PATH	  "E:/���Ե���/���ݴ���/P2C/imu381/20181020/"
//#define TEST_RAW_FILE_NAME	   "004012_raw.txt"/*����-����-�ζ�������*/
//#define TEST_RAW_FILE_PATH	  "E:/���Ե���/���ݴ���/P2C/imu381/20181022/"
//#define TEST_RAW_FILE_NAME	   "115236_raw.txt"/*԰������*/
//#define TEST_RAW_FILE_PATH	  "E:/���Ե���/���ݴ���/P2C/imu381/20181023/"
//#define TEST_RAW_FILE_NAME	   "033824_raw.txt"/*ӭ����·���ڻ��߼�*/
//#define TEST_RAW_FILE_NAME	   "060110_raw.txt"/*ӭ����·���ڻ��߼�*/
//#define TEST_RAW_FILE_NAME	   "231617_rst.txt"/*ӭ����·���ڻ��߼�,����*/
//#define TEST_RAW_FILE_PATH	  "E:/���Ե���/���ݴ���/P2C/imu381/20181024/"
//#define TEST_RAW_FILE_NAME	   "004900_raw.txt"/*ӭ����·���ڻ��߼�*/
//#define TEST_RAW_FILE_PATH	  "E:/���Ե���/���ݴ���/P2C/imu381/20181024/"
//#define TEST_RAW_FILE_NAME	   "004900_raw.txt"/*ӭ����·���ڻ��߼�*/
#define TEST_RAW_FILE_PATH	  "E:/���Ե���/���ݴ���/P2C/imu381/20181025/"
#define TEST_RAW_FILE_NAME	   "134214_raw.txt"
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

#define GILC_NMEA_OUT_PERIOD_MS    100     /*dsf90: ���10Hz*/
