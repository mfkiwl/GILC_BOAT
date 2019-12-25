#pragma once
#include "GILC_IMU_Dev.h"
#include "GILC_Boat_lib.h"

/*-------------------------------IMU TYPE-----------------------------*/
#define IMU_DEV_TYPE               IMU_DEV_ADIS16465
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

/*----------------------------TEST RAW FILE-------------------------*/
#ifdef WIN32
#define WORK_PATH  "E:/惯性导航/数据处理/"
#else
#define WORK_PATH  "/mnt/gilc-data/"
#endif

//#define TEST_RAW_FILE_PATH	 WORK_PATH"20180830_P2_ADIS16465/"
//#define TEST_RAW_FILE_NAME	   "070320_raw.txt"/*参数标定样例*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465/20180925/"
//#define TEST_RAW_FILE_NAME	   "010912_raw.txt"/*迎宾三路—内环高架*/
//#define TEST_RAW_FILE_NAME	   "073614_raw.txt"/*迎宾三路—华徐-华翔，导航应用事业部测试*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465/20180927/"
//#define TEST_RAW_FILE_NAME	   "022808_raw.txt"/*迎宾三路—华徐-华翔，导航应用事业部测试*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465/20180928/"
//#define TEST_RAW_FILE_NAME	   "035153_raw.txt"/*迎宾三路—华徐-华翔，导航应用事业部测试*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465/20180929/"
//#define TEST_RAW_FILE_NAME	   "p2_raw.txt"/*迎宾三路—华徐-华翔，导航应用事业部测试*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465/20180930/"
//#define TEST_RAW_FILE_NAME	   "044423_raw.txt"/*迎宾三路—华徐-华翔，导航应用事业部测试*/
//#define TEST_RAW_FILE_NAME	   "050133_raw.txt"/*迎宾三路—华徐-华翔，导航应用事业部测试*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465/20181008/"
//#define TEST_RAW_FILE_NAME	   "082425_raw.txt"/*迎宾三路—华徐-华翔，导航应用事业部测试*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465/20181022/"
//#define TEST_RAW_FILE_NAME	   "071430_raw.txt"/*迎宾三路—华徐-华翔，导航应用事业部测试*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465/20181023/"
//#define TEST_RAW_FILE_NAME	   "032329_raw.txt"/*迎宾三路—内环高架*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_1234567/20181121/"   /*西安，韩老师，里程计测试*/
//#define TEST_RAW_FILE_NAME	   "085351_raw.txt"
//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_1234567/20181122/"
//#define TEST_RAW_FILE_NAME	   "053802_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2C/adis16465_3181207/20181228/"   
//#define TEST_RAW_FILE_NAME	   "091135_raw.txt"            

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/adis16465_3666668/20190219/"
//#define TEST_RAW_FILE_NAME	   "051711_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/adis16465_3666670/20190222/"  //陕汽测试
//#define TEST_RAW_FILE_NAME	   "072737_raw.txt"
//#define TEST_RAW_FILE_NAME	   "084313_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/20190222cgi-h测试/71/"
//#define TEST_RAW_FILE_NAME	   "070413_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/陕汽测试_3666670/20190223/"       //陕汽测试
////#define TEST_RAW_FILE_NAME	   "020843_raw.txt"
////#define TEST_RAW_FILE_NAME	   "060932_raw.txt"
//#define TEST_RAW_FILE_NAME	   "073643_raw.txt"
////#define TEST_RAW_FILE_NAME	   "081502_raw_Bw20Hz.txt"
//#define CFG_fIns2GnssVector_X    0.000000
//#define CFG_fIns2GnssVector_Y    -2.100000
//#define CFG_fIns2GnssVector_Z    2.000000
//#define CFG_WHEEL_TRACK  2.1f
//#define CFG_WHEEL_BASE   3.9f

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/adis16465_3666670/20190223/部分/"  //陕汽测试
//#define TEST_RAW_FILE_NAME	   "062828_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/adis16465_3666669/20190308/"
//#define TEST_RAW_FILE_NAME	   "011031_raw.txt"
//#define TEST_RAW_FILE_NAME	   "053357_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/adis16465_3666669/20190311/"
//#define TEST_RAW_FILE_NAME	   "064256_raw.txt"
//#define TEST_RAW_FILE_NAME	   "072718_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/baidu_20190311/"
//#define TEST_RAW_FILE_NAME	   "035329_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/adis16465_3666670/20190318/"
//#define TEST_RAW_FILE_NAME	   "015827_raw.txt"
//#define TEST_RAW_FILE_NAME	   "025256_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/adis16465_3666667/20190318/"
//#define TEST_RAW_FILE_NAME	   "021320_raw.txt"
//#define TEST_RAW_FILE_NAME	   "025257_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"P2DF/千寻_20190322/20190322/"
//#define TEST_RAW_FILE_NAME	   "031544_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-610/4.15同济大学-清扫车/3666672/无里程计/"
//#define TEST_RAW_FILE_NAME	   "042907_raw - 2.txt"
////#define TEST_RAW_FILE_NAME	   "042907_raw - 1.txt"
//#define CFG_fIns2GnssVector_X    0.000000
//#define CFG_fIns2GnssVector_Y    -0.210000
//#define CFG_fIns2GnssVector_Z    0.830000
//#define CFG_WHEEL_TRACK  0.93f
//#define CFG_WHEEL_BASE   1.76f
//#define CFG_WORK_MODE    GILC_WORK_MODE__CAR_SLOW

//#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-610/4.15同济大学-清扫车/3666672/里程计/"
//#define TEST_RAW_FILE_NAME	   "055605_raw.txt"
//#define TEST_RAW_FILE_NAME	   "064616_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-610/4.26/74_odo/"
//#define TEST_RAW_FILE_NAME	   "114904_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-610/CGI-610千寻测试/3666675/"  /*IMU 异常*/
//#define TEST_RAW_FILE_NAME	   "022636_raw_short.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-610/里程计测试/20190512_ODO/"
//#define TEST_RAW_FILE_NAME	   "020336_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-610/里程计测试_CPT_后处理/20190508_ODO_CPT/SN92/"
//#define TEST_RAW_FILE_NAME	   "043155_raw.txt"
//#define TEST_RAW_FILE_NAME	   "043155_raw - all.txt"

#define TEST_RAW_FILE_PATH	 WORK_PATH"CGI-610/里程计测试_CPT_后处理/20190508_ODO_CPT/SN92/"
//#define TEST_RAW_FILE_NAME	   "043155_raw - all.txt"
#define TEST_RAW_FILE_NAME	   "043155_raw_cut.txt"
//#define TEST_RAW_FILE_NAME	   "043155_raw.txt"

//#undef WORK_PATH
//#define WORK_PATH  "D:/数据处理/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH"宇通客车-测试/3666692/20190513/"
//#define CFG_fIns2GnssVector_X    0.580000
//#define CFG_fIns2GnssVector_Y    -0.680000
//#define CFG_fIns2GnssVector_Z    1.000000
//#define TEST_RAW_FILE_NAME	   "030254_raw.txt"
//#define TEST_RAW_FILE_NAME	   "071024_raw.txt"                        /*里程计，未采集到有效档位信息*/
//#define TEST_RAW_FILE_NAME	   "122115_raw.txt"                        /*里程计，未采集到有效档位信息*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"5-15/91标定慢/"     /*异常数据标定测试*/
//#define TEST_RAW_FILE_NAME	 "095755_raw.txt"

//#define TEST_RAW_FILE_PATH	 WORK_PATH"新石器_20190524/"   
////#define TEST_RAW_FILE_NAME	 "034056_raw.txt"
////#define TEST_RAW_FILE_NAME	 "034056_raw_cut.txt"
//#define TEST_RAW_FILE_NAME	 "072610_raw_cut.txt"
//#define CFG_fIns2GnssVector_X    0.000000f
//#define CFG_fIns2GnssVector_Y    -0.020000f
//#define CFG_fIns2GnssVector_Z    0.830000f
//#define CFG_WHEEL_TRACK  0.93f
//#define CFG_WHEEL_BASE   1.76f
//#define CFG_fGnss2Outpoint_X  -0.250000f
//#define CFG_fGnss2Outpoint_Y  -0.020000f
//#define CFG_fGnss2Outpoint_Z  0.0f
//#define CFG_WORK_MODE    GILC_WORK_MODE__CAR_SLOW

//#define TEST_RAW_FILE_PATH	 WORK_PATH"深圳数翔-集装箱大卡-四轮转向-超低速/20190525/"     /*异常数据标定测试*/
//#define TEST_RAW_FILE_NAME	 "053425_raw.txt"
//#define TEST_RAW_FILE_NAME	 "070855_raw.txt"
//#define TEST_RAW_FILE_NAME	 "090857_raw.txt"
//#define TEST_RAW_FILE_NAME	 "111325_raw.txt"   /*GNSS-异常*/

//#define TEST_RAW_FILE_PATH	 WORK_PATH"深圳数翔-集装箱大卡-四轮转向-超低速/20190526/" 
////#define TEST_RAW_FILE_NAME	 "072625_raw.txt"
//#define TEST_RAW_FILE_NAME	 "075209_raw.txt"
//#define CFG_fIns2GnssVector_X    -0.027000
//#define CFG_fIns2GnssVector_Y    0.877300
//#define CFG_fIns2GnssVector_Z    0.499700
//#define CFG_WHEEL_TRACK  1.908000
//#define CFG_WHEEL_BASE   11.017100
//#define CFG_fIns2GnssAngle_Z 180
//#define CFG_WORK_MODE    GILC_WORK_MODE__CAR_SLOW
//#define CFG_fIns2BodyAngleErr_Z  1

////#define TEST_RAW_FILE_PATH	 WORK_PATH"北京兴科迪-大矿车-中低速/20190610/" 
////#define TEST_RAW_FILE_NAME	 "020956_raw_cut.txt"
//#define TEST_RAW_FILE_PATH	 WORK_PATH"北京兴科迪-大矿车-中低速/20190616/" 
//#define TEST_RAW_FILE_NAME	 "010325_raw.txt"
////#define TEST_RAW_FILE_NAME	 "060226_raw.txt"
////#define TEST_RAW_FILE_NAME	 "062154_raw.txt"
////#define TEST_RAW_FILE_NAME	 "071625_raw.txt"
//#define CFG_WHEEL_TRACK  5.283000
//#define CFG_WHEEL_BASE   4.800000
//#define CFG_fIns2GnssVector_X    -0.760000
//#define CFG_fIns2GnssVector_Y    3.040000
//#define CFG_fIns2GnssVector_Z    2.210000
//#define CFG_fIns2GnssAngle_Z -90
//#define CFG_WORK_MODE    GILC_WORK_MODE__CAR_SLOW

//#define TEST_RAW_FILE_PATH	 WORK_PATH"深圳-顺丰-物流小车/20190605/"   
////#define TEST_RAW_FILE_NAME	 "024025_raw.txt"
//#define TEST_RAW_FILE_NAME	 "024725_raw.txt"
////#define CFG_fIns2GnssVector_X     0.420000
////#define CFG_fIns2GnssVector_Y     0.830000
////#define CFG_fIns2GnssVector_Z     0.800000
//#define CFG_WHEEL_TRACK  0.7f
//#define CFG_WHEEL_BASE   0.9f
//#define CFG_fIns2GnssAngle_Z  90
//#define CFG_WORK_MODE    GILC_WORK_MODE__CAR_SLOW

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
