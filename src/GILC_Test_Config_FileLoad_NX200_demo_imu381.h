#pragma once
#include "GILC_IMU_Dev.h"
#include "GILC_Boat_lib.h"

#include <math.h>

/*-------------------------------IMU BW(Hz)-----------------------------*/
#define IMU_BW_HZ                  10

/*--------------------------------IMU WALK-----------------------------*/
/*TUpdate(,1)*/
#define GYRO_WALK (0.3*0.05)
#define VEL_WALK (5000)

/*--------------------------------IMU STD-----------------------------*/
#define GYRO_STD_X (0.011*sqrt(IMU_BW_HZ))
#define ACC_STD_X (0.000150*sqrt(IMU_BW_HZ))

#define GYRO_STD_Y GYRO_STD_X
#define GYRO_STD_Z GYRO_STD_X

#define ACC_STD_Y ACC_STD_X
#define ACC_STD_Z ACC_STD_X

/*--------------------------------IMU AXIS-----------------------------*/
#define CFG_GYRO_X_SCALE  (1)
#define CFG_GYRO_Y_SCALE  (1)
#define CFG_GYRO_Z_SCALE (-1)
#define CFG_ACC_X_SCALE   (1)
#define CFG_ACC_Y_SCALE   (1)
#define CFG_ACC_Z_SCALE  (-1)

#define CFG_GYRO_X_ROW 1
#define CFG_GYRO_Y_ROW 2
#define CFG_GYRO_Z_ROW 3
#define CFG_ACC_X_ROW 4
#define CFG_ACC_Y_ROW 5
#define CFG_ACC_Z_ROW 6
/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_fIns2GnssVector_X    0.0
#define CFG_fIns2GnssVector_Y    0.0
#define CFG_fIns2GnssVector_Z    0.0

/*----------------------------TEST RAW FILE-------------------------*/
#ifdef WIN32
#define WORK_PATH  "E:/惯性导航/数据处理/"
#else
#define WORK_PATH  "/mnt/gilc-data/"
#endif

#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-610/nx200-data5/"
#define TEST_RAW_FILE_NAME	   "063750_raw.txt"
/*--------------------------------TEST_OUT/TMP_FILE_PATH-----------------------------*/
#define TEST_OUT_FILE_PATH    TEST_RAW_FILE_PATH"OUT/"
#define TEST_TMP_FILE_PATH    "D:/TMP/"

/*--------------------------------TEST_RST_FILE_NAME----------------------------*/
#define TEST_RST_FILE_NAME     "gilc_vehicle_rst.nmea"

/*--------------------------------RAW CFG-----------------------------*/
#define CFG_POS_STD_USE 0
#define CFG_VEL_STD_USE 0
#define CFG_GNSSPOS_MODE GILC_GNSSPOS_MODE__LLA_RAD
#define CFG_GNSSVEL_MODE GILC_GNSSVEL_MODE__SPEED_HEADING

#define GILC_NMEA_OUT_PERIOD_MS    100     /*dsf90: 输出10Hz*/
