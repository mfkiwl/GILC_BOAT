#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include "gnss_nmea.h"
/*----------------------------------------------
���룺
dataBuf  ���ݵ�ַ
len      ���ݳ���
���أ�
1�ֽ������
----------------------------------------------*/
HC_INT8 NMEA_Check(HC_VOID *dataBuf,HC_UINT32 len)
{
	HC_INT8 *data = (HC_INT8 *)dataBuf;
	HC_INT8 sum;
	HC_UINT32 i;
	if(!data||!len)
		return 0;
	sum = *data;
	for(i=1;i<len;i++)
		sum ^= *(data+i);
	return sum;
}

//��buf����õ���cx���������ڵ�λ��
//����ֵ:0~0XFE,����������λ�õ�ƫ��.
//       0XFF,�������ڵ�cx������							  
HC_UINT8 NMEA_Comma_Pos(HC_UINT8 *buf,HC_UINT8 cx)
{	 		    
	HC_UINT8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
		if(*buf==',')cx--;
		buf++;
	}
	if(*buf==',') /*dsf90:����','���ڣ�����Ϊ��*/
		return 0XFF; 
	return buf-p;	 
}
//m^n����
//����ֵ:m^n�η�.
HC_UINT32 NMEA_Pow(HC_UINT8 m,HC_UINT8 n)
{
	HC_UINT32 result=1;	 
	while(n--)result*=m;    
	return result;
}
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
int NMEA_Str2num(HC_UINT8 *buf,HC_UINT8*dx)
{
	HC_UINT8 *p=buf;
	HC_UINT32 ires=0,fres=0;
	HC_UINT8 ilen=0,flen=0,i;
	HC_UINT8 mask=0;
	int res;
	while(1) //�õ�������С���ĳ���
	{
		if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
		if(*p==','||(*p=='*'))break;//����������
		if(*p=='.'){mask|=0X01;p++;}//����С������
		else if(*p>'9'||(*p<'0'))	//�зǷ��ַ�
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//ȥ������
	for(i=0;i<ilen;i++)	//�õ�������������
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//���ȡ5λС��
	*dx=flen;	 		//С����λ��
	for(i=0;i<flen;i++)	//�õ�С����������
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 
//����GPGSV��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GSV_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p,*p1,dx;
	HC_UINT8 len,i,j,slx=0;
	HC_UINT8 posx; 
	
	gpsx->svnum = 0;
	gpsx->snsum = 0;
	p=buf;
	
	while(1)
	{	
		//p1=(u8*)strstr((const char *)p,"$GPGSV");
		//len=p1[7]-'0';								//�õ�GPGSV������
		p1=(HC_UINT8 *)strstr((const char *)p,"GSV");
		if(!p1) 	return;
		
		len=p1[4]-'0';								//�õ�GPGSV������
		//posx=NMEA_Comma_Pos(p1,3); 					//�õ��ɼ���������
		//if(posx!=0XFF) gpsx->svnum +=NMEA_Str2num(p1+posx,&dx);
		for(i=0;i<len;i++)
		{	 
			//p1=(HC_UINT8*)strstr((const char *)p,"$GPGSV");  
			p1=(HC_UINT8*)strstr((const char *)p,"GSV");  
			if (!p1)
				return;
			for(j=0;j<4;j++)
			{	  
				posx=NMEA_Comma_Pos(p1,4+j*4);
				if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//�õ����Ǳ��
				else break; 
				posx=NMEA_Comma_Pos(p1,5+j*4);
				if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//�õ��������� 
				else break;
				posx=NMEA_Comma_Pos(p1,6+j*4);
				if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//�õ����Ƿ�λ��
				else break; 
				posx=NMEA_Comma_Pos(p1,7+j*4);
				if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//�õ����������
				else break;
				if(gpsx->slmsg[slx].sn>0) 										//�������Ч
				{
					gpsx->svnum ++;
					gpsx->snsum +=gpsx->slmsg[slx].sn;
				}
				slx++;	 
			}   
	 		p=p1+1;//�л�����һ��GPGSV��Ϣ
		}  		
	}
}
//����GPGGA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GGA_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p1,dx;			 
	HC_UINT8 posx;    
	HC_UINT32 temp,temp2;    
	HC_INT32 iTemp;
	//p1=(HC_UINT8*)strstr((const char *)buf,"$GPGGA");
	p1=(HC_UINT8*)strstr((const char *)buf,"GGA");
	if(!p1)		return;
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp2=NMEA_Str2num(p1+posx,&dx);	 	//�õ�UTCʱ��
		temp = temp2/NMEA_Pow(10,dx);
		gpsx->utc.time= (double)temp2/NMEA_Pow(10,dx);
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp2%10000;	
	}		
	posx=NMEA_Comma_Pos(p1,6);								//�õ�GPS״̬
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//�õ����ڶ�λ��������
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	posx=NMEA_Comma_Pos(p1,9);								//�õ����θ߶�
	if(posx!=0XFF)
	{
		iTemp =NMEA_Str2num(p1+posx,&dx);
		gpsx->altitude=(HC_FLOAT)iTemp/NMEA_Pow(10,dx);	
	}
	posx=NMEA_Comma_Pos(p1,11);								//�õ����ˮ׼��߶�
	if(posx!=0XFF)
	{
		iTemp =NMEA_Str2num(p1+posx,&dx);
		gpsx->height_wgs84 = (HC_FLOAT)iTemp/NMEA_Pow(10,dx);
	}
	gpsx->height = gpsx->altitude + gpsx->height_wgs84;
	
	posx=NMEA_Comma_Pos(p1,13);								//�����ʱ
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);  
		gpsx->dgnss_timeout = (HC_FLOAT)temp/NMEA_Pow(10,dx);
	}
	
	posx=NMEA_Comma_Pos(p1,14);								//��վID
	if(posx!=0XFF) gpsx->station_id=NMEA_Str2num(p1+posx,&dx);  

}
//����GPGSA��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_GSA_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p1,dx;			 
	HC_UINT8 posx; 
	HC_UINT8 i;  
	HC_UINT32 temp;
	//p1=(HC_UINT8*)strstr((const char *)buf,"$GPGSA");
	p1=(HC_UINT8*)strstr((const char *)buf,"GSA");
	if(!p1)		return;
	posx=NMEA_Comma_Pos(p1,2);								//�õ���λ����
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//�õ���λ���Ǳ��
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//�õ�PDOPλ�þ�������
	if (posx != 0XFF)
	{
		temp = NMEA_Str2num(p1 + posx, &dx);
		gpsx->pdop = (HC_FLOAT)temp / NMEA_Pow(10, dx);
	}
	posx=NMEA_Comma_Pos(p1,16);								//�õ�HDOPλ�þ�������
	if (posx!=0XFF)
	{
		temp = NMEA_Str2num(p1 + posx, &dx);
		gpsx->hdop = (HC_FLOAT)temp / NMEA_Pow(10, dx);
	}
	posx=NMEA_Comma_Pos(p1,17);								//�õ�VDOPλ�þ�������
	if (posx != 0XFF)
	{
		temp = NMEA_Str2num(p1 + posx, &dx);
		gpsx->vdop = (HC_FLOAT)temp / NMEA_Pow(10, dx);
	}
}
//����GPRMC��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_RMC_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p1,dx;			 
	HC_UINT8 posx;     
	HC_UINT32 temp,temp2;	   
	HC_DOUBLE rs;  
	//p1=(HC_UINT8*)strstr((const char *)buf,"GPRMC");//"$GPRMC",������&��GPRMC�ֿ������,��ֻ�ж�GPRMC.
	p1=(HC_UINT8*)strstr((const char *)buf,"RMC");//"$GPRMC",������&��GPRMC�ֿ������,��ֻ�ж�GPRMC.
	if(!p1)		return;
	posx=NMEA_Comma_Pos(p1,1);								//�õ�UTCʱ��
	if(posx!=0XFF)
	{
		temp2=NMEA_Str2num(p1+posx,&dx);	 	//�õ�UTCʱ��
		temp = temp2/NMEA_Pow(10,dx);
		gpsx->utc.time=(double)temp2/NMEA_Pow(10,dx);
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp2%10000;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//�õ�γ��
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->latitude=gpsx->latitude+rs/NMEA_Pow(10,dx)/60.0;//ת��Ϊ�� 
		if(gpsx->latitude > 0)
			gpsx->count++;
		else if (gpsx->latitude == 0)
			gpsx->count = 0;
	}
	posx=NMEA_Comma_Pos(p1,4);								//��γ���Ǳ�γ 
	if(posx!=0XFF && (*(p1+posx)=='N' || *(p1+posx)=='S' ))
		gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//�õ�����
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//�õ���
		rs=temp%NMEA_Pow(10,dx+2);				//�õ�'		 
		gpsx->longitude=gpsx->longitude+(HC_DOUBLE)rs/NMEA_Pow(10,dx)/60.0;//ת��Ϊ�� 
	}
	posx=NMEA_Comma_Pos(p1,6);								//������������
	if(posx!=0XFF && (*(p1+posx)=='E' || *(p1+posx)=='W' ))
		gpsx->ewhemi=*(p1+posx);		 
	posx=NMEA_Comma_Pos(p1,7);								//�ٶ�
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		gpsx->speed = (HC_FLOAT)temp / NMEA_Pow(10, dx);
		gpsx->speed = (HC_UINT32)(gpsx->speed*1.852f/3.6f);	//����/Сʱ -> m/s
	}
	posx=NMEA_Comma_Pos(p1,8);								//�ٶ�
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);
		gpsx->heading=(HC_FLOAT)temp/NMEA_Pow(10,dx);	 	 
	}
	posx=NMEA_Comma_Pos(p1,9);								//�õ�UTC����
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 				//�õ�UTC����
		if(temp>0)
		{
			gpsx->utc.date=temp/10000;
			gpsx->utc.month=(temp/100)%100;
			gpsx->utc.year=2000+temp%100;	 	 
			//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
			//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */      
			gpsx->utc.fattime = (((HC_UINT32)gpsx->utc.year-1980)&0x0000003F)<<25;			
			gpsx->utc.fattime |= ((HC_UINT32)gpsx->utc.month&0x0000000F)<<21;			
			gpsx->utc.fattime |= ((HC_UINT32)gpsx->utc.date&0x0000001F)<<16;			
			gpsx->utc.fattime |= ((HC_UINT32)gpsx->utc.hour&0x0000001F)<<11;			
			gpsx->utc.fattime |= ((HC_UINT32)gpsx->utc.min&0x0000003F)<<5;			
			gpsx->utc.fattime |= ((HC_UINT32)(gpsx->utc.sec/100/2)&0x0000001F);	
		}
	} 
}
//����GPVTG��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void NMEA_VTG_Analysis(nmea_msg *gpsx,HC_UINT8 *buf)
{
	HC_UINT8 *p1,dx;			 
	HC_UINT8 posx;  
	HC_UINT32 temp;
	//p1=(HC_UINT8*)strstr((const char *)buf,"$GPVTG");							 
	p1=(HC_UINT8*)strstr((const char *)buf,"VTG");							 
	if(!p1)		return;
	posx=NMEA_Comma_Pos(p1,7);								//�õ���������
	if(posx!=0XFF)
	{
		temp = NMEA_Str2num(p1 + posx, &dx);
		gpsx->speed = (HC_FLOAT)temp / NMEA_Pow(10, dx);
		gpsx->speed = (HC_UINT32)(gpsx->speed*1.852f / 3.6f);	//����/Сʱ -> m/s
	}
}  
//��ȡNMEA-0183��Ϣ
//gpsx:nmea��Ϣ�ṹ��
//buf:���յ���GPS���ݻ������׵�ַ
void GPS_Analysis(HC_UINT8 *buf,HC_UINT16 len,nmea_msg *gpsx)
{
	NMEA_RMC_Analysis(gpsx,buf);	//GPRMC����
	NMEA_GGA_Analysis(gpsx,buf);	//GPGGA���� 	
	NMEA_GSA_Analysis(gpsx,buf);	//GPGSA����
	NMEA_GSV_Analysis(gpsx,buf);	//GPGSV����
	//NMEA_VTG_Analysis(gpsx,buf);	//GPVTG����
	/*
	HC_UINT8 i;
	printf("svnum\t\t:%u\r\n", gpsx->svnum);
	printf("latitude\t:%.7f\r\n", gpsx->latitude);
	printf("nshemi\t\t:%c\r\n", gpsx->nshemi);
	printf("longitude\t:%u\r\n", gpsx->longitude);
	printf("ewhemi\t\t:%c\r\n", gpsx->ewhemi);
	printf("gpssta\t\t:%u\r\n", gpsx->gpssta);
	printf("posslnum\t:%u\r\n", gpsx->posslnum);
	printf("possl\t\t:");
	for(i=0;i<gpsx->posslnum;i++)
		printf("%u  ", gpsx->possl[i]);
	printf("\r\n");
	printf("fixmode\t\t:%u\r\n", gpsx->fixmode);
	printf("pdop\t\t:%u\r\n", gpsx->pdop);
	printf("hdop\t\t:%u\r\n", gpsx->hdop);
	printf("vdop\t\t:%u\r\n", gpsx->vdop);
	printf("height\t\t:%d\r\n", gpsx->height);
	printf("speed\t\t:%u\r\n", gpsx->speed);
	printf("heading\t\t:%u\r\n", gpsx->heading);
	*/
}

