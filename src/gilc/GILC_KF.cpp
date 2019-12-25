/*add by dsf90, 2019.2.12*/
#ifdef WIN32
#include "stdafx.h"
#include <string.h>
#endif

#include "math.h"
#include "GILC.h"
#include "GILC_KF.h"
#include "GILC_Config.h"
#define MUP_TEST_DSF90 1

#define KF_DEBUG

void TUpdate_True(int ROW,int COL,double Qk[],double Gk[],double Phi[], double xk[], double Pxk[], double dt,int opt)
{
	if(!opt)
	{
		double* temp=(double*)__ml_zero(sizeof(double)*ROW*1);
		double* temp1=(double*)__ml_zero(sizeof(double)*ROW*ROW);
		double* temp2=(double*)__ml_zero(sizeof(double)*ROW*ROW);

		/*dsf90:xk  = Phi X xk*/
		Mmulnm(Phi,xk,ROW,ROW,1,temp);
		Mequalm(temp,ROW,1,xk);
		Mmulnm(Phi,Pxk,ROW,ROW,ROW,temp1);
		Mtn(Phi,ROW,ROW,temp2);
		Mmulnm(temp1,temp2,ROW,ROW,ROW,Pxk);
	#if 0
		/*dsf90:Pxk = Phi X Pxk X Mt(Phi) + Qk*dt*/
		Mmuln(Qk,ROW,ROW,dt,temp1);
		Madd(Pxk,temp1,ROW,ROW);
	#else   
		/*dsf90:������Pxk = Phi X Pxk X Mt(Phi) + Qk*dt^2*/
		Mmuln(Qk,ROW,ROW,dt*dt,temp1);
		Madd(Pxk,temp1,ROW,ROW);
	#endif
		gilc_free(temp);  temp=NULL;
		gilc_free(temp1); temp1=NULL;
		gilc_free(temp2); temp2=NULL;	
	}
	else  // ����GK
	{
		double* temp = (double*)__ml_zero(sizeof(double)*ROW * 1);
		double* temp1 = (double*)__ml_zero(sizeof(double)*ROW*ROW);
		double* temp2 = (double*)__ml_zero(sizeof(double)*ROW*ROW);
		double* temp3 = (double*)__ml_zero(sizeof(double)*ROW*ROW);

		/*dsf90:xk  = Phi X xk*/
		Mmulnm(Phi, xk, ROW, ROW, 1, temp);
		Mequalm(temp, ROW, 1, xk);
		Mmulnm(Phi, Pxk, ROW, ROW, ROW, temp1);
		Mtn(Phi, ROW, ROW, temp2);
		Mmulnm(temp1, temp2, ROW, ROW, ROW, Pxk);
	#if 0
		/*dsf90:Pxk = Phi X Pxk X Mt(Phi) + Gk X (Qk*dt) X Gk*/
		Mmuln(Qk, ROW, ROW, dt, temp1);
		Mmulnm(Gk, temp1, ROW, ROW, ROW, temp2);
		Mtn(Gk, ROW, ROW, temp1);
		Mmulnm(temp2, Gk, ROW, ROW, ROW, temp3);
	#else
		/*dsf90:������Pxk = Phi X Pxk X Mt(Phi) + dt^2 * Gk X Qk X Mt(Gk)*/
		Mmuln(Qk, ROW, ROW, (dt*dt), temp1);
		Mmulnm(Gk, temp1, ROW, ROW, ROW, temp2);
		Mtn(Gk, ROW, ROW, temp1);
		Mmulnm(temp2, temp1, ROW, ROW, ROW, temp3);
	#endif
		Madd(Pxk, temp3, ROW, ROW);

		gilc_free(temp);   temp = NULL;
		gilc_free(temp1);  temp1 = NULL;
		gilc_free(temp2);  temp2 = NULL;
		gilc_free(temp3);  temp3 = NULL;
	}
}

void MUpdate_True(int ROW,int COL,double Hk[],double Rk[],double ZK[], double xk[], double Pxk[])
{
	double* xkk_1=(double*)__ml_zero(sizeof(double)*ROW*1);
	double* Pxkk_1=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	double* Pxykk_1=(double*)__ml_zero(sizeof(double)*ROW*COL);
	double* Pykk_1=(double*)__ml_zero(sizeof(double)*COL*COL);
	double* Kk=(double*)__ml_zero(sizeof(double)*ROW*COL);
	double* ykk_1=(double*)__ml_zero(sizeof(double)*COL*1);
	double* dxk=(double*)__ml_zero(sizeof(double)*ROW*1);
	double* Hkt=(double*)__ml_zero(sizeof(double)*ROW*COL);
	double* invPy=(double*)__ml_zero(sizeof(double)*COL*COL);
	double* KPyKt=(double*)__ml_zero(sizeof(double)*ROW*ROW);

	/*dsf90:Phi = F*/
	/*dsf90:xk	= X*/
	/*dsf90:Pxk	= P*/
	/*dsf90:Hk	= H*/
	/*dsf90:ZK	= Z*/
	/*dsf90:Rk	= R*/

	Mequalm(xk,ROW,1,xkk_1);
	Mequalm(Pxk,ROW,ROW,Pxkk_1);
	
	/*dsf90:Pxykk_1	= Pxk X Mt(Hk)*/
	/*dsf90:Pykk_1	= Hk X Pxk X Mt(Hk) + Rk*/
	/*dsf90:ykk_1	= ZK - Hk X xk*/
	/*dsf90:dxk	    = Kk X (ZK - Hk X xk)*/
	
	/*dsf90:Kk	    = Pxk X Mt(Hk) X inv(Hk X Pxk X Mt(Hk) + Rk)*/
	/*dsf90:xk  	= xk + Kk X (ZK - Hk X xk)*/
		
	/*dsf90:Hkt	= Mt(Kk)*/
	/*dsf90:Pxykk_1	= Kk X (Hk X Pxk X Mt(Hk) + Rk)*/
	/*dsf90:KPyKt	= Kk X (Hk X Pxk X Mt(Hk) + Rk) X Mt(Hk)*/
	/*dsf90:Pxk  	= Pxk - Kk X (Hk X Pxk X Mt(Hk) + Rk) X Mt(Hk)*/
	/*dsf90:Pxk  	= 0.5*(Pxk + Mt(Pxk))*/

	Mtn(Hk,COL,ROW,Hkt);
	Mmulnm(Pxkk_1,Hkt,ROW,ROW,COL,Pxykk_1); 
	Mmulnm(Hk,Pxykk_1,COL,ROW,COL,Pykk_1);
	Madd(Pykk_1,Rk,COL,COL); 
	Minvn(Pykk_1,COL,invPy);
	Mmulnm(Pxykk_1,invPy,ROW,COL,COL,Kk); 
	Mmulnm(Hk,xkk_1,COL,ROW,1,ykk_1); 
	Mminn(ZK,ykk_1,ykk_1,COL,1);
	Mmulnm(Kk,ykk_1,ROW,COL,1,dxk);
	Maddn(xkk_1,dxk,xk,ROW,1); 
#if (!MUP_TEST_DSF90)
	Mtn(Kk,ROW,COL,Hkt);
	Mmulnm(Kk,Pykk_1,ROW,COL,COL,Pxykk_1);
	Mmulnm(Pxykk_1,Hkt,ROW,COL,ROW,KPyKt);
	Mminn(Pxkk_1,KPyKt,Pxk,ROW,ROW); 
	Mtn(Pxk,ROW,ROW,Pxkk_1);
	Madd(Pxk,Pxkk_1,ROW,ROW);
	//Mmul(Pxk,ROW,ROW,0.5*0.995); 
	Mmul(Pxk,ROW,ROW,0.5); 
#else
	/*add by dsf90,2018,3,1;����Ч��һ�£�û�в���*/
	double* KkHk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	double* KkHkPxkk_1=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	
	/*dsf90:Pxk  	= Pxk - Kk X Hk X Pxk*/
	Mmulnm(Kk,Hk,ROW,COL,ROW,KkHk);
	Mmulnm(KkHk,Pxkk_1,ROW,ROW,ROW,KkHkPxkk_1);
	Mminn(Pxkk_1,KkHkPxkk_1,Pxk,ROW,ROW); 
	//Mmul(Pxk,ROW,ROW,0.995);
	
	gilc_free(KkHk);       KkHk=NULL;
	gilc_free(KkHkPxkk_1); KkHkPxkk_1=NULL;
	/*end add*/
#endif	
	gilc_free(xkk_1); 
	gilc_free(Pxkk_1);
	gilc_free(Pxykk_1);
	gilc_free(Pykk_1);
	gilc_free(Kk);
	gilc_free(ykk_1);
	gilc_free(dxk);
	gilc_free(Hkt);
	gilc_free(invPy);
	gilc_free(KPyKt);
	xkk_1=NULL;
	Pxkk_1=NULL;
	Pxykk_1=NULL;
	Pykk_1=NULL;
	Kk=NULL;
	ykk_1=NULL;
	dxk=NULL;
	Hkt=NULL;
	invPy=NULL;
	KPyKt=NULL;
}

void MUpdate_Variable(int ROW,int COL,double Hk[],double Rk[],double ZK[], double xk[], double Pxk[], int zflag)
{
	int i=0,j=0,k=0,usecol = 0;
	for(i=0;i<COL;i++)
	{
		if((zflag>>i)%2)
			usecol ++;
	}
	double* usehk=(double*)__ml_zero(sizeof(double)*usecol*ROW);
	double* userk=(double*)__ml_zero(sizeof(double)*usecol*usecol);
	double* usezk=(double*)__ml_zero(sizeof(double)*usecol*1);
	for(i=0;i<COL;i++)
	{
		if((zflag>>i)%2)
		{
			for(j=0;j<ROW;j++)
				usehk[k*ROW+j] = Hk[i*ROW + j];	
			userk[k*usecol+k] = Rk[i];
			usezk[k] = ZK[i];
			k++;
		}
	}

	MUpdate_True(ROW,usecol,usehk,userk,usezk,xk,Pxk);

	gilc_free(usehk);
	gilc_free(userk);
	gilc_free(usezk);
}

GIKF_Calibrate::GIKF_Calibrate(int row,int col)
{
	ROW=row;
	COL=col;
	xflag=0xffffffff;
	zflag=0xffffffff;
	xk=(double*)__ml_zero(sizeof(double)*ROW*1);
	xk_std = (double*)__ml_zero(sizeof(double)*ROW * 1);
	Pxk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Phi=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Qk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Gk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Hk=(double*)__ml_zero(sizeof(double)*COL*ROW);
	Rk=(double*)__ml_zero(sizeof(double)*COL*COL);
}

//��������������ڴ� ��ֵ
GIKF_Calibrate& GIKF_Calibrate::operator=(const GIKF_Calibrate& kftemp)
{
	ROW=kftemp.ROW;
	COL=kftemp.COL;
	xflag=kftemp.xflag;
	zflag=kftemp.zflag;
	for(int i=0;i<ROW;i++)
	{
		*(xk+i)=*(kftemp.xk+i);
	}
	for(int i=0;i<ROW*ROW;i++)
	{
		*(Pxk+i)=*(kftemp.Pxk+i);
		*(Phi+i)=*(kftemp.Phi+i);
		*(Qk+i)=*(kftemp.Qk+i);
		*(Gk+i)=*(kftemp.Gk+i);
	}
	for(int i=0;i<COL*ROW;i++)
	{*(Hk+i)=*(kftemp.Hk+i);}
	for(int i=0;i<COL*COL;i++)
	{*(Rk+i)=*(kftemp.Rk+i);}
	return(*this);
}

void GIKF_Calibrate::kfinit()
{
	xflag = 0xffffffff;
	zflag = 0xffffffff;
	
	memset(xk,0,sizeof(double)*ROW*1);
	memset(xk_std,0,sizeof(double)*ROW*1);
	memset(Phi,0,sizeof(double)*ROW*ROW);
	memset(Hk,0,sizeof(double)*COL*ROW);
	memset(Rk,0,sizeof(double)*COL*1);
	memset(Pxk,0,sizeof(double)*ROW*ROW);
	memset(Qk,0,sizeof(double)*ROW*ROW);
	memset(Gk,0,sizeof(double)*ROW*ROW);
	
	Rk[0] = SQR(InitParam.dKfInitP_OdoHeading);
	Rk[1] = SQR(InitParam.dKfInitP_OdoVel);
	Rk[2] = SQR(InitParam.dKfInitP_DualYaw);
	
	/*dsf90:Pxk ��ʼ��Ϊ����*/
	Pxk[0 * ROW + 0] = SQR(0.01);
	//Pxk[0*ROW+0]=SQR(0.01);
	//Pxk[1*ROW+1]=SQR(0.5);
	Pxk[2*ROW+2]=SQR(10);  
	
	Pxk[3*ROW+3]=SQR(0.01);    
	Pxk[4*ROW+4]=SQR(10);   
	
	Pxk[5*ROW+5]=SQR(0.01);    
	Pxk[6*ROW+6]=SQR(10);   
	
	/*dsf90:��������*/
	Qk[2*ROW+2] = SQR(0.1);
	Qk[4*ROW+4] = SQR(0.1);
	Qk[6*ROW+6] = SQR(0.1);

	Gk[2*ROW+2]=1.0;
	Gk[4*ROW+4]=1.0;
	Gk[6*ROW+6]=1.0;
}

void GIKF_Calibrate::kffree()
{
	if(xk!=NULL)
	{gilc_free(xk);xk=NULL;}
	if(xk_std!=NULL)
	{gilc_free(xk_std);xk_std=NULL;}
	if(Phi!=NULL)
	{gilc_free(Phi);Phi=NULL;}
	if(Hk!=NULL)
	{gilc_free(Hk);Hk=NULL;}
	if(Pxk!=NULL)
	{gilc_free(Pxk);Pxk=NULL;}
	if(Qk!=NULL)
	{gilc_free(Qk);Qk=NULL;}
	if(Gk!=NULL)
	{gilc_free(Gk);Gk=NULL;}
	if(Rk!=NULL)
	{gilc_free(Rk);Rk=NULL;}
}

void GIKF_Calibrate::upPhi(CSINS& ins,double dt,int iType)
{
	Mequal(Phi,ROW,ROW,0);
	if (iType&CAL_UPDATE_TYPE_WHEEL_HEADING)
	{
		/*������ת��*/
		Phi[2 * ROW + 1] = ins.wheel_heading_h; Phi[2 * ROW + 2] = 1;
	}
	if (iType&CAL_UPDATE_TYPE_WHEEL_VEL)
	{
		/*��̼�����*/
		Phi[4*ROW+3]=ins.wheel_vel_h; 
	}
	if (iType&CAL_UPDATE_TYPE_DUAL_YAW)
	{
		/*˫���ߺ���*/
		Phi[6 * ROW + 5] = 1;
	}

	/*dsf90:Phi = I + F*T = I + Phi*dt */
	double *eye=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Munit(eye,ROW);
	Mmul(Phi,ROW,ROW,dt);
	Madd(Phi,eye,ROW,ROW);
	gilc_free(eye);
}

void GIKF_Calibrate::upHk(CSINS& ins, double Hk[])
{
	/*dsf90:������ת��*/
	if (ins.Kwh)
	{
		Hk[0 * ROW + 0] = (ins.Bwh - ins.wheel_heading) / SQ(ins.Kwh);
		Hk[0 * ROW + 1] = -1 / ins.Kwh;
		Hk[0 * ROW + 2] = 1 / ins.Kwh;
	}
	/*dsf90:��̼�����*/
	if (ins.Kd)
	{
		Hk[1 * ROW + 3] = -ins.wheel_vel / SQ(ins.Kd);
		Hk[1 * ROW + 4] = 1 / ins.Kd;
	}
	/*dsf90:˫���ߺ���*/
	Hk[2 * ROW + 5] = -1;
	Hk[2 * ROW + 6] =  1;
}

void GIKF_Calibrate::TUpdate(double dt,int opt,int iType)
{
	Mequal(Qk, ROW, ROW, 0);
	if (iType&CAL_UPDATE_TYPE_WHEEL_HEADING)
	{
		/*������ת��*/
		Qk[2*ROW+2] = SQR(0.1);
	}
	if (iType&CAL_UPDATE_TYPE_WHEEL_VEL)
	{
		/*��̼�����*/
		Qk[4 * ROW + 4] = SQR(0.1);
	}
	if (iType&CAL_UPDATE_TYPE_DUAL_YAW)
	{
		/*˫���ߺ���*/
		Qk[6*ROW+6] = SQR(0.1);
	}

	TUpdate_True(ROW,COL,Qk,Gk,Phi,xk,Pxk,dt, opt);
}

void GIKF_Calibrate::MUpdate(CSINS& ins, double ZK[], int zflag)
{
	double* hk = (double*)__ml_zero(sizeof(double)*COL*ROW);
	upHk(ins,hk);
	MUpdate_Variable(ROW, COL, hk, Rk, ZK, xk, Pxk, zflag);
#ifdef KF_DEBUG
	static FILE *fp = fopen("./Pxk.txt", "wb");
	if (!fp)
		gilc_log("open file err! ./Pxk.txt\n");
	else
		fprintf(fp, "Pxk %.8f %.8f %.8f %.8f %.8f %.8f %.8f\r\n", Pxk[0], Pxk[1* ROW+1], Pxk[2 * ROW + 2], Pxk[3 * ROW + 3], Pxk[4 * ROW + 4], Pxk[5 * ROW + 5], Pxk[6 * ROW + 6]);
#endif

	gilc_free(hk);
}

void GIKF_Calibrate::Feedback(CSINS& ins,double scater)
{
	double* xk_f = (double*)__ml_zero(sizeof(double)*ROW);
	
	Mmuln(xk,ROW,1,scater,xk_f);
	/*dsf90:������ת��*/
	ins.Kwh += xk_f[0];
	ins.Bwh += xk_f[1];
	ins.wheel_heading += xk_f[2];
	/*dsf90:��̼�����*/
	ins.Kd += xk_f[3];
	ins.wheel_vel += xk_f[4];
	/*dsf90:˫���ߺ���*/
	ins.Byaw += xk_f[5];
	ins.dual_yaw += xk_f[6];

	for(int i=0;i<ROW;i++)
	{
		xk[i] -= xk_f[i];
		xk_std[i] = sqrt(Pxk[i*ROW+i]);
	}
#ifdef KF_DEBUG
	static FILE *fxk_f = fopen("./xk_f.txt", "wb");
	if (!fxk_f)
		gilc_log("open file err! ./xk_f.txt\n");
	else
		fprintf(fxk_f,"xk_f %.8f %.8f %.8f %.8f %.8f %.8f %.8f\r\n",xk_f[0],xk_f[1],xk_f[2],xk_f[3],xk_f[4],xk_f[5],xk_f[6]);
#endif
	gilc_free(xk_f);
}

GIKF_Calibrate_DualAnt::GIKF_Calibrate_DualAnt(int row,int col)
{
	ROW=row;
	COL=col;
	xflag=0xffffffff;
	zflag=0xffffffff;
	xk=(double*)__ml_zero(sizeof(double)*ROW*1);
	xk_std = (double*)__ml_zero(sizeof(double)*ROW * 1);
	Pxk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Phi=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Qk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Gk=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Hk=(double*)__ml_zero(sizeof(double)*COL*ROW);
	Rk=(double*)__ml_zero(sizeof(double)*COL*COL);
}

//��������������ڴ� ��ֵ
GIKF_Calibrate_DualAnt& GIKF_Calibrate_DualAnt::operator=(const GIKF_Calibrate_DualAnt& kftemp)
{
	ROW=kftemp.ROW;
	COL=kftemp.COL;
	xflag=kftemp.xflag;
	zflag=kftemp.zflag;
	for(int i=0;i<ROW;i++)
	{
		*(xk+i)=*(kftemp.xk+i);
	}
	for(int i=0;i<ROW*ROW;i++)
	{
		*(Pxk+i)=*(kftemp.Pxk+i);
		*(Phi+i)=*(kftemp.Phi+i);
		*(Qk+i)=*(kftemp.Qk+i);
		*(Gk+i)=*(kftemp.Gk+i);
	}
	for(int i=0;i<COL*ROW;i++)
	{*(Hk+i)=*(kftemp.Hk+i);}
	for(int i=0;i<COL*COL;i++)
	{*(Rk+i)=*(kftemp.Rk+i);}
	return(*this);
}

void GIKF_Calibrate_DualAnt::kfinit()
{
	xflag = 0xffffffff;
	zflag = 0xffffffff;
	
	memset(xk,0,sizeof(double)*ROW*1);
	memset(xk_std,0,sizeof(double)*ROW*1);
	memset(Phi,0,sizeof(double)*ROW*ROW);
	memset(Hk,0,sizeof(double)*COL*ROW);
	memset(Rk,0,sizeof(double)*COL*1);
	memset(Pxk,0,sizeof(double)*ROW*ROW);
	memset(Qk,0,sizeof(double)*ROW*ROW);
	memset(Gk,0,sizeof(double)*ROW*ROW);
	
	Rk[1] = SQR(InitParam.dKfInitP_DualYaw);
	
	/*dsf90:Pxk ��ʼ��Ϊ����*/
	Pxk[0*ROW+0]=SQR(1);    
	Pxk[1*ROW+1]=SQR(100);   
	
	/*dsf90:��������*/
	Qk[1*ROW+1] = SQR(0.1);

	Gk[1*ROW+1]=1.0;
}

void GIKF_Calibrate_DualAnt::kffree()
{
	if(xk!=NULL)
	{gilc_free(xk);xk=NULL;}
	if(xk_std!=NULL)
	{gilc_free(xk_std);xk_std=NULL;}
	if(Phi!=NULL)
	{gilc_free(Phi);Phi=NULL;}
	if(Hk!=NULL)
	{gilc_free(Hk);Hk=NULL;}
	if(Pxk!=NULL)
	{gilc_free(Pxk);Pxk=NULL;}
	if(Qk!=NULL)
	{gilc_free(Qk);Qk=NULL;}
	if(Gk!=NULL)
	{gilc_free(Gk);Gk=NULL;}
	if(Rk!=NULL)
	{gilc_free(Rk);Rk=NULL;}
}

void GIKF_Calibrate_DualAnt::upPhi(CSINS& ins,double dt)
{
	Mequal(Phi,ROW,ROW,0);
	/*˫���ߺ���*/
	Phi[2*ROW+0]=1; 

	/*dsf90:Phi = I + F*T = I + Phi*dt */
	double *eye=(double*)__ml_zero(sizeof(double)*ROW*ROW);
	Munit(eye,ROW);
	Mmul(Phi,ROW,ROW,dt);
	Madd(Phi,eye,ROW,ROW);
	gilc_free(eye);
}

void GIKF_Calibrate_DualAnt::upHk(CSINS& ins, double Hk[])
{
	/*dsf90:˫���ߺ���*/
	Hk[0 * ROW + 0] = -1;
	Hk[0 * ROW + 1] =  1;
}

void GIKF_Calibrate_DualAnt::TUpdate(double dt,int opt)
{
	TUpdate_True(ROW,COL,Qk,Gk,Phi,xk,Pxk,dt, opt);
}

void GIKF_Calibrate_DualAnt::MUpdate(CSINS& ins, double ZK[], int zflag)
{
	double* hk = (double*)__ml_zero(sizeof(double)*COL*ROW);
	upHk(ins,hk);
	MUpdate_Variable(ROW, COL, hk, Rk, ZK, xk, Pxk, zflag);
	gilc_free(hk);
}

void GIKF_Calibrate_DualAnt::Feedback(CSINS& ins,double scater)
{
	double* xk_f = (double*)__ml_zero(sizeof(double)*ROW);
	
	Mmuln(xk,ROW,1,scater,xk_f);
	/*dsf90:˫���ߺ���*/
	ins.Byaw += xk_f[0];
	ins.dual_yaw += xk_f[1];

	for(int i=0;i<ROW;i++)
	{
		xk[i] -= xk_f[i];
		xk_std[i] = sqrt(Pxk[i*ROW+i]);
	}
	gilc_free(xk_f);
}

