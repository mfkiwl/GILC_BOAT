#ifndef __GNSS_NMEA_H__
#define __GNSS_NMEA_H__

#include "hc_type.h"

#ifdef __cplusplus
extern "C" {
#endif				/*__cplusplus*/

////////////////////////////////////////////////////////////////////////////////// 	   


//GPS NMEA-0183Э����Ҫ�����ṹ�嶨�� 
//������Ϣ
typedef struct  
{										    
 	HC_UINT8 num;		//���Ǳ��
	HC_UINT8 eledeg;	//��������
	HC_UINT16 azideg;	//���Ƿ�λ��
	HC_UINT8 sn;		//�����		   
}nmea_slmsg;  
//UTCʱ����Ϣ
typedef struct  
{										    
	HC_DOUBLE time; //utctime
	HC_UINT16 year;	//���
	HC_UINT8 month;	//�·�
	HC_UINT8 date;	//����
	HC_UINT8 hour; 	//Сʱ
	HC_UINT8 min; 	//����
	HC_UINT32 sec; 	//����100��,ʵ�ʳ���100.	
	HC_UINT32 sec_ms; 	//����1000��.	
	//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                      
	//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
	HC_UINT32 fattime; /*�ļ�ϵͳʱ���ʽ*/
}nmea_utc_time; 

typedef struct
{
	HC_DOUBLE sec; //������
	HC_UINT16 week;	//GPS-��
}nmea_gps_time;

typedef struct  
{										    
	HC_UINT8 insused_flag;
	HC_UINT8 ins_out_type;
	HC_UINT8 ins_status;
	HC_DOUBLE rpy[3];
	HC_DOUBLE gyo[3];/*deg/s*/
	HC_DOUBLE acc[3];/*g*/
	HC_UINT8 gilcstatus;
	HC_UINT8 warming;
}nmea_ins_msg;   	   

typedef struct  
{										    
	HC_UINT8 chip_id[16];
	HC_UINT16 chip_type; /*407*/
}nmea_sys_msg;   	   

//NMEA 0183 
typedef struct  
{		
	/*GGA msg*/
	nmea_utc_time utc;			//UTCʱ��
	nmea_gps_time gps_time;
	HC_DOUBLE latitude;				//γ�� 
	HC_UINT8 nshemi;					//��γ/��γ,N:��γ;S:��γ				  
	HC_DOUBLE longitude;			    //���� 
	HC_UINT8 ewhemi;					//����/����,E:����;W:����
	HC_UINT8 gpssta;					//GPS״̬:0,δ��λ;1,����;2,RTD;4,RTK;5,����;6,���ڹ���.				  
 	HC_UINT8 posslnum;				//���ڶ�λ��������,0~16.
	HC_DOUBLE hdop;					//ˮƽ�������� 
	HC_DOUBLE altitude;			 	      //���θ߶�,	 
	HC_DOUBLE height_wgs84;		      //���ˮ׼��߶� 
	HC_DOUBLE dgnss_timeout;		      //�����ʱ	 
	HC_UINT16 station_id;		              //��վ���.	 
	
 	HC_UINT8 possl[32];				//���ڶ�λ�����Ǳ��
	HC_UINT8 fixmode;					//��λ����:1,û�ж�λ;2,2D��λ;3,3D��λ
	HC_DOUBLE pdop;					//λ�þ�������
	HC_DOUBLE vdop;					//��ֱ��������

	HC_DOUBLE height;                //�߶�altitude +  height_wgs84
	HC_DOUBLE biseline;
	HC_UINT8 nsused;				//ʹ������
	HC_UINT8 nsused2;				//ʹ������
	HC_UINT8 svnum;					//�ɼ�������
 	HC_UINT16 snsum;                                 //�������������֮��
	nmea_slmsg slmsg[32];		                //���32������

	HC_DOUBLE speed;					//�������� ��λ:1����/Сʱ
	HC_DOUBLE ve;                        //�����ٶ�
	HC_DOUBLE vn;                        //�����ٶ�
	HC_DOUBLE vd;                        //�����ٶ�
	
	HC_DOUBLE heading;					
	HC_DOUBLE heading2;					
	HC_UINT32 count;					

	HC_DOUBLE sigma_lat;                /*����*/
	HC_DOUBLE sigma_lon;
	HC_DOUBLE sigma_alt;
	HC_DOUBLE sigma_ve;
	HC_DOUBLE sigma_vn;
	HC_DOUBLE sigma_vd;

	nmea_ins_msg ins_msg;
	nmea_sys_msg sys_msg;
}nmea_msg; 

void GPS_Analysis(HC_UINT8 *buf,HC_UINT16 len,nmea_msg *gpsx);
void NMEA_GSV_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);
void NMEA_GGA_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);
void NMEA_GSA_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);
void NMEA_RMC_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);
void NMEA_VTG_Analysis(nmea_msg *gpsx,HC_UINT8 *buf);


#ifdef __cplusplus
}
#endif			/*__cplusplus*/

#endif			/*__GNSS_NMEA_H__*/

