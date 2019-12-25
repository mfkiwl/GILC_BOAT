/*
 * GILC_Config.h
 *
 *  Created on: May 16, 2018
 *      Author: dsf
 */

#pragma once

#ifndef STM32
#include "stdio.h"
#endif
/*------------
日志：
v0.4.0_20190515
  1、开放里程计车速接入功能
v0.4.0_20190519
  1、标定/初始化结果添加阈值约束，解决IMU加速度计异常，仍能初始化成功问题；
  2、20分钟不能完成初始化，认为初始化失败，添加初始化失败返回值
v0.4.1_20190525（解决深圳低速集装箱运输车辆标定不成功问题）
  1、初始化成功前，生效双天线航向观测；
  2、初始化成功前，按照双天线航向误差配置参数设置双天线观测噪声；
v0.4.1_20190604
  添加GILC_Cfg_OutReferPoint配置接口，实现输出位置实时修改，无需重新初始化
v0.4.2_20190615
  1、解决双天线横向安装，设别上点航向显示异常问题
  2、解决GNSS跳点判断数据源选择错误
v0.4.2_20190616
  1、车辆静止不适用双天线航向策略修改
 v0.4.3_20190702
  1、适配SCC2X30村田方案
 v0.4.4_20190705
  1、初步实现标定后，上电静止初始化策略
  2、生效pstOut->iSensorFlag 部分组合信息
  3、修改linux部分编译错误
 v0.4.4_20190711
  1、算法库log,支持打印等级设置
  2、算法库log,支持实体函数callback重定向
 v0.4.5_20190716
  1、修改部分警告
  2、修改GNSS数据组包卫星颗数
  3、修改双天线定向可用std阈值
-------------*/
/*--------------------------------GILC LIB VERSION-----------------------------*/
//#define GILC_SOFT_VER  "GD100.v0.0.0"		//固定状态可以自动巡航，单点自动巡航转圈，单点初始化失败
//#define GILC_SOFT_DATE "20190904"

//#define GILC_SOFT_VER  "GD100.v0.1.0"		//去掉20分钟安装标定失败条件判断，解决20分钟后自动巡航转圈的问题
//#define GILC_SOFT_DATE "20190919"


//#define GILC_SOFT_VER  "GD100.v0.1.1"		//动静态判断速度阈值0.1，静态时，加入位置更新，解决船停止时，水流飘动，算法识别不出来动态
//#define GILC_SOFT_DATE "20190919"


//#define GILC_SOFT_VER  "GD100.v0.1.2"		//解决单点时，高程变化大，自动巡航失败
//#define GILC_SOFT_DATE "20190920"

//#define GILC_SOFT_VER  "GD100.v1.0.0"		//安装误差配置文件，第一次标定3分钟+GNSS固定
//#define GILC_SOFT_DATE "20190927"

//#define GILC_SOFT_VER  "GD100.v1.3.0"		//算法初始：静态初始，实时计算俯仰、横滚
//#define GILC_SOFT_DATE "20191010"			//算法初始：单点时航向std<2deg,固定时std<1deg
//											//安装误差标定：第一次标定时间>5min
//											//安装误差标定：安装误差角std<0.15deg heading_std<1deg
//

//#define GILC_SOFT_VER  "GD100.v1.3.1"		//高程优化：高程转换到以测深仪为中心
//#define GILC_SOFT_DATE "20191010"

//#define GILC_SOFT_VER  "GD100.v1.3.2"		//加入安装误差检测，即使存在配置文件，每次仍需估计安装误差值
//#define GILC_SOFT_DATE "20191010"			//安装误差再次收敛后，与配置文件参数对比，误差大于0.2
//											//认为机器位置发生改变，需要重新写入安装误差配置参数
//											//双天线航向安装误差不作检测(安装位置固定)

//#define GILC_SOFT_VER  "GD100.v1.3.3"		//算法过程去掉静态初始，静态初始轨迹有异常
//#define GILC_SOFT_DATE "20191012"			//

//#define GILC_SOFT_VER  "GD100.v2.1.0"		//动态初始，固定时取双天线航向，单点时取速度航向
//#define GILC_SOFT_DATE "20191016"			//安装误差标定尽量固定标定，单点标定结果不理想

//#define GILC_SOFT_VER  "GD100.v2.1.1"		//动态初始，初始速度1m/s，防止岸上误初始
//#define GILC_SOFT_DATE "20191031"			//

//#define GILC_SOFT_VER  "GD100.v2.1.2"		//取消动静态识别，全程约束修正
//#define GILC_SOFT_DATE "20191113"	

//#define GILC_SOFT_VER  "GD100.v2.1.3"		//双天线安装误差设定阈值20度，防止单天线速度航向误标定成功
//#define GILC_SOFT_DATE "20191114"		    //针对双天线航向为0的情况，双天线安装误差不做修正

//#define GILC_SOFT_VER  "GD100.v2.1.4"		//最新稳定版算法库
//#define GILC_SOFT_DATE "20191116"		

//#define GILC_SOFT_VER  "GD100.v2.1.5"		//增加gnss固定时动静态判断，静态输出gnss固定位置
//#define GILC_SOFT_DATE "20191121"			//市场端需求静态测固定点，精度3cm以内

#define GILC_SOFT_VER  "GD100.v2.2.0"		//取消侧向速度不完整约束，有水流情况下，该约束不适用
#define GILC_SOFT_DATE "20191210"			//计算水流速度，作为载体侧向速度观测输入
/*-----------------------------------------------------------------------------*/

#define GILC_SIMPLIFY_USED            1
#define GILC_INSTALL_FIX              0

#define GNSS_PERIODE_MS 200

/*--------------------------------TEST_OUT_FILE_PATH-----------------------------*/
#define TEST_OUT_FILE_PATH "./GILC-Vehicle/out/"
#define TEST_TMP_FILE_PATH "./GILC-Vehicle/temp/"

/*--------------------------------IMU STD-----------------------------*/

#define GILC_GYRO_STD_X (0.003*sqrt(40.0))
#define GILC_GYRO_STD_Y GILC_GYRO_STD_X	
#define GILC_GYRO_STD_Z GILC_GYRO_STD_X

#define GILC_ACC_STD_X (0.000023*sqrt(40.0))
#define GILC_ACC_STD_Y GILC_ACC_STD_X
#define GILC_ACC_STD_Z GILC_ACC_STD_X 
/*--------------------------------IMU WALK-----------------------------*/
#define GILC_GYRO_WALK (0.15*0.2)
#define GILC_VEL_WALK (1200)
/*--------------------------------IMU RAW CFG-----------------------------*/
#define CFG_GYRO_X_SCALE (-D2R)
#define CFG_GYRO_Y_SCALE  (D2R)
#define CFG_GYRO_Z_SCALE  (D2R)
#define CFG_ACC_X_SCALE   (1)
#define CFG_ACC_Y_SCALE  (-1)
#define CFG_ACC_Z_SCALE  (-1)

#define CFG_GYRO_X_ROW 2
#define CFG_GYRO_Y_ROW 1
#define CFG_GYRO_Z_ROW 3
#define CFG_ACC_X_ROW 5
#define CFG_ACC_Y_ROW 4
#define CFG_ACC_Z_ROW 6

