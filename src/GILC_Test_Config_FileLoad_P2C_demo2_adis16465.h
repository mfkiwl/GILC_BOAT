#pragma once
#include "GILC_IMU_Dev.h"
#include "GILC_Boat_lib.h"

/*-------------------------------IMU TYPE-----------------------------*/
#include "GILC_IMU_Config_adis16465_demo2.h"

/*--------------------------------LEVER(m)-----------------------------*/
#define CFG_fIns2GnssVector_X    0.0
#define CFG_fIns2GnssVector_Y    0.0
#define CFG_fIns2GnssVector_Z    0.0

/*----------------------------TEST RAW FILE-------------------------*/
#ifdef WIN32
#define WORK_PATH  "E:/���Ե���/���ݴ���/"
#else
#define WORK_PATH  "/mnt/gilc-data/"
#endif

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181102/"
//#define TEST_RAW_FILE_NAME	   "101039_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181119/"
//#define TEST_RAW_FILE_NAME	   "073302_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871260/20181119/"
//#define TEST_RAW_FILE_NAME	   "073306_raw.txt"
//#define TEST_RAW_FILE_NAME	   "122157_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871260/20181120/"/*DR ����*/
//#define TEST_RAW_FILE_NAME	   "002037_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871260/20181121/"/*DR ����*/
//#define TEST_RAW_FILE_NAME	   "032536_raw.txt"   /*����ͬ��ʧ��*/
//#define TEST_RAW_FILE_NAME	   "040229_raw.txt"
//#define TEST_RAW_FILE_NAME	   "055845_raw.txt"
//#define TEST_RAW_FILE_NAME	   "065543_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871260/20181126/"
//#define TEST_RAW_FILE_NAME	   "040449_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871260/20181128/" /*���� ����*/
//#define TEST_RAW_FILE_NAME	   "094031_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871260/20181129/"
//#define TEST_RAW_FILE_NAME	   "012105_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871260/20181206/"
//#define TEST_RAW_FILE_NAME	   "060400_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871260/20181207/"
//#define TEST_RAW_FILE_NAME	   "022529_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181214/"
//#define TEST_RAW_FILE_NAME	   "140637_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181215/"
//#define TEST_RAW_FILE_NAME	   "071328_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181217/"       /*casco Э�����*/
//#define TEST_RAW_FILE_NAME	   "032243_raw.txt"
//#define TEST_RAW_FILE_NAME	   "080536_raw.txt"
//#define TEST_RAW_FILE_NAME	   "081659_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181215/"
//#define TEST_RAW_FILE_NAME	   "071328_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/20181205����Ӧ�ò���/20181210����P2����/414/"
//#define TEST_RAW_FILE_NAME	   "070728_raw.txt"
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/20181205����Ӧ�ò���/20181210����P2����/610/"
//#define TEST_RAW_FILE_NAME	   "070728_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/20181205����Ӧ�ò���/20181211����P2����/414/"
//#define TEST_RAW_FILE_NAME	   "062335_raw.txt"
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/20181205����Ӧ�ò���/20181211����P2����/610/"
//#define TEST_RAW_FILE_NAME	   "062332_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/20181205����Ӧ�ò���/20181212����P2����/414/"
//#define TEST_RAW_FILE_NAME	   "085059_raw.txt"
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/20181205����Ӧ�ò���/20181212����P2����/610/"
//#define TEST_RAW_FILE_NAME	   "081031_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/20181205����Ӧ�ò���/20181213����P2����/414/"
//#define TEST_RAW_FILE_NAME	   "082002_raw.txt"
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/20181205����Ӧ�ò���/20181213����P2����/610/"
//#define TEST_RAW_FILE_NAME	   "082450_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181225/"    /*��װ�ǶȲ��ԣ���峯�£����߿ڳ���(0,180,90)*/
//#define TEST_RAW_FILE_NAME	   "110536_raw.txt"                  
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181226/"    /*��װ�ǶȲ��ԣ���峯�£����߿ڳ���(0,180,90)*/
//#define TEST_RAW_FILE_NAME	   "110648_raw.txt"                  
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181227/"    /*��װ�ǶȲ��ԣ���峯�£����߿ڳ���(0,180,90)*/
//#define TEST_RAW_FILE_NAME	   "235822_raw.txt"                  
//#define TEST_RAW_FILE_NAME	   "025511_raw.txt"                  
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3871007/20181228/"    /*��װ�ǶȲ��ԣ���峯�ϣ����߿ڳ���(0,0,-90)*/
//#define TEST_RAW_FILE_NAME	   "091137_raw.txt"                  
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3206414/20190102/"    /*��װ�ǶȲ��ԣ���峯�ϣ����߿ڳ���(0,0,-90)*/
//#define TEST_RAW_FILE_NAME	   "065640_raw.txt"                  
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3872610/20190102/"    /*��װ�ǶȲ��ԣ���峯�ϣ����߿ڳ���(0,0,-90)*/
//#define TEST_RAW_FILE_NAME	   "065642_raw.txt"                  
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3206414/20190103/"    /*��װ�ǶȲ��ԣ���峯�ϣ����߿ڳ���(0,0,-90)*/
//#define TEST_RAW_FILE_NAME	   "030822_raw.txt"                  
//#define TEST_RAW_FILE_NAME	   "062231_raw.txt"     

#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/20190108�Ȳ�/1007_20190108/"    
#define TEST_RAW_FILE_NAME	   "030532_raw.txt"                  

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

