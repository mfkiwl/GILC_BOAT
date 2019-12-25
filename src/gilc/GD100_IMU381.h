/*
* TestFileConfig.h
*
*  Created on: May 15, 2018
*      Author: dsf
*/

#ifndef _GILC_IMU_CONFIG_H_
#define _GILC_IMU_CONFIG_H_

#include <math.h>

/*-------------------------------IMU BW(Hz)-----------------------------*/
#define IMU_BW_HZ                  40

/*--------------------------------IMU WALK-----------------------------*/
/*TUpdate(,1)*/
#define GYRO_WALK (0.3*0.05)
#define VEL_WALK (3000)

/*--------------------------------IMU STD-----------------------------*/
#define GYRO_STD_X (0.011*sqrt(IMU_BW_HZ))  //0.011
#define ACC_STD_X (0.00025*sqrt(IMU_BW_HZ)) //0.00005

#define GYRO_STD_Y GYRO_STD_X
#define GYRO_STD_Z GYRO_STD_X

#define ACC_STD_Y ACC_STD_X
#define ACC_STD_Z ACC_STD_X



/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_fIns2GnssVector_X    0.0
#define CFG_fIns2GnssVector_Y    -0.1
#define CFG_fIns2GnssVector_Z    0.25
/*--------------------------------IMU AXIS-----------------------------*/
#define CFG_GYRO_X_SCALE  (-1)
#define CFG_GYRO_Y_SCALE  (1)
#define CFG_GYRO_Z_SCALE (1)
#define CFG_ACC_X_SCALE   (-1)
#define CFG_ACC_Y_SCALE   (1)
#define CFG_ACC_Z_SCALE  (1)

#define CFG_GYRO_X_ROW 2
#define CFG_GYRO_Y_ROW 1
#define CFG_GYRO_Z_ROW 3
#define CFG_ACC_X_ROW 5
#define CFG_ACC_Y_ROW 4
#define CFG_ACC_Z_ROW 6
//#define CFG_GYRO_X_SCALE  (1)
//#define CFG_GYRO_Y_SCALE  (1)
//#define CFG_GYRO_Z_SCALE (1)
//#define CFG_ACC_X_SCALE   (1)
//#define CFG_ACC_Y_SCALE   (1)
//#define CFG_ACC_Z_SCALE  (1)
//
//#define CFG_GYRO_X_ROW 1
//#define CFG_GYRO_Y_ROW 2
//#define CFG_GYRO_Z_ROW 3
//#define CFG_ACC_X_ROW 4
//#define CFG_ACC_Y_ROW 5
//#define CFG_ACC_Z_ROW 6
#define GILC_NMEA_OUT_FLAG         0x0007  

//#define WORK_PATH  "G:/GD100项目/测试数据/9.12外场测试/20190912/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "082948_raw.txt"


//#define WORK_PATH  "G:/GD100项目/测试数据/11.08/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "072912_raw.txt"


//#define WORK_PATH  "G:/GD100项目/测试数据/11.12/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "075250_raw.txt"

//#define WORK_PATH  "G:/GD100项目/测试数据/11.15/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "072629_raw.txt"

//#define WORK_PATH  "G:/GD100项目/测试数据/11.04/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "083520_raw.txt"

//#define WORK_PATH  "G:/GD100项目/测试数据/9.27/20190927/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "093357_raw.txt"

//#define WORK_PATH  "G:/GD100项目/测试数据/9.29外场测试/20190929/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "083847_raw.txt"


//#define WORK_PATH  "G:/GD100项目/测试数据/9.18外场测试/data3/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "083856_raw.txt"

//#define WORK_PATH  "G:/GD100项目/测试数据/8.13外场测试/data1/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "101207_raw.txt"


//#define WORK_PATH  "G:/GD100项目/测试数据/8.7外场测试/data1/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "094025_raw.txt"


//#define WORK_PATH  "C:/Users/huace/Desktop/827/data1/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "104056_raw.txt"


//#define WORK_PATH  "G:/GD100项目/测试数据/9.03园区测试/data1/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "074034_raw.txt"

//#define WORK_PATH  "G:/GD100项目/测试数据/11.19数据/1119/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "072843_raw.txt"


//#define WORK_PATH  "G:/GD100项目/测试数据/11.20/1120/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "072147_raw.txt"

//#define WORK_PATH  "G:/GD100项目/测试数据/12.3/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	   "033007_raw.txt"

#define WORK_PATH  "G:/GD100项目/测试数据/12.11/1209航向偏移/"
#define TEST_RAW_FILE_PATH	 WORK_PATH
#define TEST_RAW_FILE_NAME	   "131412_raw.txt"
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

#endif
