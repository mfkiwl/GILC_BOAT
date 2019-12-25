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
#define CFG_fIns2GnssVector_Y    -0.9
#define CFG_fIns2GnssVector_Z    0.3

/*----------------------------TEST RAW FILE-------------------------*/
#ifdef WIN32
//#define WORK_PATH  "G:/GD100项目/测试数据/8.1园区测试/"
#define WORK_PATH  "G:/GD100项目/测试数据/7.30外场测试/"
#else
#define WORK_PATH  "/mnt/gilc-data/"
#endif

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181017/"
//#define TEST_RAW_FILE_NAME	   "011610_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181019/"
//#define TEST_RAW_FILE_NAME	   "060137_raw.txt"/*园区车库*/
//#define TEST_RAW_FILE_NAME	   "063038_raw.txt"/*迎宾三路-松泽-华徐-园区车库*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181020/"
//#define TEST_RAW_FILE_NAME	   "004012_raw.txt"/*徐泾-青浦-嘉定，高速*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181022/"
//#define TEST_RAW_FILE_NAME	   "115236_raw.txt"/*园区车库*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181023/"
//#define TEST_RAW_FILE_NAME	   "033824_raw.txt"/*迎宾三路—内环高架*/
//#define TEST_RAW_FILE_NAME	   "060110_raw.txt"/*迎宾三路—内环高架*/
//#define TEST_RAW_FILE_NAME	   "231617_raw.txt"/*迎宾三路—内环高架------------单点*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181024/"
//#define TEST_RAW_FILE_NAME	   "004900_raw.txt"/*迎宾三路—内环高架*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181029/"
//#define TEST_RAW_FILE_NAME	   "232459_raw.txt"/*迎宾三路—内环高架*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181030/"
//#define TEST_RAW_FILE_NAME	   "113637_raw.txt"/*公寓——园区，标定*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181031/"
//#define TEST_RAW_FILE_NAME	   "101805_raw.txt"/*园区——家乐福，标定*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181102/"
//#define TEST_RAW_FILE_NAME	   "005109_raw.txt"/*OBD测试*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181106/"
//#define TEST_RAW_FILE_NAME	   "130643_raw.txt"/*进程优先级提高优化测试*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/imu381/20181107/"
//#define TEST_RAW_FILE_NAME	   "122144_raw.txt"/*进程优先级提高优化+实时内核 测试*/

#define TEST_RAW_FILE_PATH	 WORK_PATH
#define TEST_RAW_FILE_NAME	   "092654_raw.txt"/*进程优先级提高优化+实时内核+关闭wifi 测试*/

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
