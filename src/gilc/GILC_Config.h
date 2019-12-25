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
��־��
v0.4.0_20190515
  1��������̼Ƴ��ٽ��빦��
v0.4.0_20190519
  1���궨/��ʼ����������ֵԼ�������IMU���ٶȼ��쳣�����ܳ�ʼ���ɹ����⣻
  2��20���Ӳ�����ɳ�ʼ������Ϊ��ʼ��ʧ�ܣ���ӳ�ʼ��ʧ�ܷ���ֵ
v0.4.1_20190525��������ڵ��ټ�װ�����䳵���궨���ɹ����⣩
  1����ʼ���ɹ�ǰ����Ч˫���ߺ���۲⣻
  2����ʼ���ɹ�ǰ������˫���ߺ���������ò�������˫���߹۲�������
v0.4.1_20190604
  ���GILC_Cfg_OutReferPoint���ýӿڣ�ʵ�����λ��ʵʱ�޸ģ��������³�ʼ��
v0.4.2_20190615
  1�����˫���ߺ���װ������ϵ㺽����ʾ�쳣����
  2�����GNSS�����ж�����Դѡ�����
v0.4.2_20190616
  1��������ֹ������˫���ߺ�������޸�
 v0.4.3_20190702
  1������SCC2X30���﷽��
 v0.4.4_20190705
  1������ʵ�ֱ궨���ϵ羲ֹ��ʼ������
  2����ЧpstOut->iSensorFlag ���������Ϣ
  3���޸�linux���ֱ������
 v0.4.4_20190711
  1���㷨��log,֧�ִ�ӡ�ȼ�����
  2���㷨��log,֧��ʵ�庯��callback�ض���
 v0.4.5_20190716
  1���޸Ĳ��־���
  2���޸�GNSS����������ǿ���
  3���޸�˫���߶������std��ֵ
-------------*/
/*--------------------------------GILC LIB VERSION-----------------------------*/
//#define GILC_SOFT_VER  "GD100.v0.0.0"		//�̶�״̬�����Զ�Ѳ���������Զ�Ѳ��תȦ�������ʼ��ʧ��
//#define GILC_SOFT_DATE "20190904"

//#define GILC_SOFT_VER  "GD100.v0.1.0"		//ȥ��20���Ӱ�װ�궨ʧ�������жϣ����20���Ӻ��Զ�Ѳ��תȦ������
//#define GILC_SOFT_DATE "20190919"


//#define GILC_SOFT_VER  "GD100.v0.1.1"		//����̬�ж��ٶ���ֵ0.1����̬ʱ������λ�ø��£������ֹͣʱ��ˮ��Ʈ�����㷨ʶ�𲻳�����̬
//#define GILC_SOFT_DATE "20190919"


//#define GILC_SOFT_VER  "GD100.v0.1.2"		//�������ʱ���̱߳仯���Զ�Ѳ��ʧ��
//#define GILC_SOFT_DATE "20190920"

//#define GILC_SOFT_VER  "GD100.v1.0.0"		//��װ��������ļ�����һ�α궨3����+GNSS�̶�
//#define GILC_SOFT_DATE "20190927"

//#define GILC_SOFT_VER  "GD100.v1.3.0"		//�㷨��ʼ����̬��ʼ��ʵʱ���㸩�������
//#define GILC_SOFT_DATE "20191010"			//�㷨��ʼ������ʱ����std<2deg,�̶�ʱstd<1deg
//											//��װ���궨����һ�α궨ʱ��>5min
//											//��װ���궨����װ����std<0.15deg heading_std<1deg
//

//#define GILC_SOFT_VER  "GD100.v1.3.1"		//�߳��Ż����߳�ת�����Բ�����Ϊ����
//#define GILC_SOFT_DATE "20191010"

//#define GILC_SOFT_VER  "GD100.v1.3.2"		//���밲װ����⣬��ʹ���������ļ���ÿ��������ư�װ���ֵ
//#define GILC_SOFT_DATE "20191010"			//��װ����ٴ��������������ļ������Աȣ�������0.2
//											//��Ϊ����λ�÷����ı䣬��Ҫ����д�밲װ������ò���
//											//˫���ߺ���װ�������(��װλ�ù̶�)

//#define GILC_SOFT_VER  "GD100.v1.3.3"		//�㷨����ȥ����̬��ʼ����̬��ʼ�켣���쳣
//#define GILC_SOFT_DATE "20191012"			//

//#define GILC_SOFT_VER  "GD100.v2.1.0"		//��̬��ʼ���̶�ʱȡ˫���ߺ��򣬵���ʱȡ�ٶȺ���
//#define GILC_SOFT_DATE "20191016"			//��װ���궨�����̶��궨������궨���������

//#define GILC_SOFT_VER  "GD100.v2.1.1"		//��̬��ʼ����ʼ�ٶ�1m/s����ֹ�������ʼ
//#define GILC_SOFT_DATE "20191031"			//

//#define GILC_SOFT_VER  "GD100.v2.1.2"		//ȡ������̬ʶ��ȫ��Լ������
//#define GILC_SOFT_DATE "20191113"	

//#define GILC_SOFT_VER  "GD100.v2.1.3"		//˫���߰�װ����趨��ֵ20�ȣ���ֹ�������ٶȺ�����궨�ɹ�
//#define GILC_SOFT_DATE "20191114"		    //���˫���ߺ���Ϊ0�������˫���߰�װ��������

//#define GILC_SOFT_VER  "GD100.v2.1.4"		//�����ȶ����㷨��
//#define GILC_SOFT_DATE "20191116"		

//#define GILC_SOFT_VER  "GD100.v2.1.5"		//����gnss�̶�ʱ����̬�жϣ���̬���gnss�̶�λ��
//#define GILC_SOFT_DATE "20191121"			//�г�������̬��̶��㣬����3cm����

#define GILC_SOFT_VER  "GD100.v2.2.0"		//ȡ�������ٶȲ�����Լ������ˮ������£���Լ��������
#define GILC_SOFT_DATE "20191210"			//����ˮ���ٶȣ���Ϊ��������ٶȹ۲�����
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

