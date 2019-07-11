//cordic算法

#pragma warning(disable: 4996)
#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "csv_operation.h"
#include "struct_Init.h"


#define DBL_EPSILON     2.2204460492503131e-025 /* smallest such that 1.0+DBL_EPSILON != 1.0 */
#define EXIT_SUCCESS    0
#define EXIT_FAILURE    1




struct eth eth0 = {

	/*Re 参数已经设为常量*/
	.e2 = 0.00669437999,
	.wie = 0.000072921151467,
	.sl = 0.97526633331,
	.cl = -0.221032981962205,
	.tl = -4.412311342183346,
	.sl2 = 0.951144420884896,
	.RNh = 6398680.704312623,
	.clRNh = -1414169.476698240,
	.RMh = 6396574.601487093,
	.wnie = { 0, -0.161179795568686e-04, 0.711175440118507e-04 },
	.wnen = { 0.312667345353095e-08,0.156282216008374e-08,-0.689565794275296e-08 },
	.wnien = { 0.000003126673454e-03 , 	-0.032234396291577e-03 ,0.142228192365759e-03 },
	.wnin = { 0.000031266734535e-04,- 0.161164167347085e-04,0.711106483539080e-04 },
};


struct data_updata data_updata0 =
{
	.minute = 0,
	.second = 0.0,
	.wx = 0.0,
	.wy = 0.0,
	.wz = 0.0,
	.ax = 0.0,
	.ay = 0.0,
	.az = 0.0,

	.pitch = 0.0,
	.roll = 0.0,
	.yaw = 0.0,

	.mx = 0.0,
	.my = 0.0,
	.mz = 0.0,

	.pressure = 0.0,
	.satellite_flag = 0,
	.hight = 0.0,
	.high = 0.0,
	.lat = 0.0,
	.lon = 0.0,

	.vx = 0.0,
	.vy = 0.0,
	.vz = 0.0,
	.heading = 0.0,
	.UTC = 0,
};




/*
 * 常数
 */
int    cnt       =  0;
double pi	     =  3.1415926535;
double g	     =  9.87032;
double Re		 =  6378137;        //地球长半径

double latitude  = 45.7796;			//纬度
double longitude = 126.6705;	    //经度

double deg2rad = 0;			    //并非真值
double rad2deg = 0;				//并非真值
		

/*
 * 机械常数,列向量
 */
double wdb[3] = { 0.1, 0.1, 0.1 };		//angular random walk分辨率


/*
 * 读取数据
 * 1. 引进数据
 * 2. minute要记得减7
 * 3. 
 */
int nn = 2;		//采样数
double ts = 0.0209; //采样间隔
double nts = 0.02;
double h = 0.1;
double p = 0.1;

/*
 * 惯导,gps初始误差,全都是常值,但是全都是列向量
 */
double pma[3] = { 30, -30, 20 };
double dvn[3] = { 0.5, 0.5, 0.5 };
double dpos0[3] = { 0.1, 0.1, 0.1 };
double dpos[3] = { 0.000000023517839, 0.000000023517839, 0.1 };
double davp[9] = { 0.000145444104333, -0.000145444104333, 0.005817764173314, 0.5, 0.5, 0.5, 0.000000023517839, 0.000000023517839, 0.1 };

/*
 * Kalman滤波的第一组参数,也可以设置为变量,也可以设置为常数,这些都是列向量
 */
double att0[3];
double vn0[3];
double pos0[3];
double avp0[9];

/*
 * Kalman滤波的更新参数,这些肯定是变量,会随着变化
 */
double att[3];
double vn[3];
double pos[3];
double avp[9];
double qnb0[4]; // qnb0 = Cbp2q(attitude2Cbp(avp0(1:3))); %*deg2rad)); 
double qnb[4];

double Cnb0[3][3];
double Cnb[3][3];

/*
 * Kalman 滤波的参数
 */
double Q[15][15] = { 0 };	/*这两个参数很重要*/
double R[6][6] = { 0 };

double P[15][15] = { 0 };
double H[6][15] = { 0 };

double xk[15] = { 0 };
double K[15][6] = { 0 };

double Mpv[3][3] = { 0, 0.000000156333673, 0, -0.000000707128825, 0, 0,	0 ,0 ,1};










void parameter_init(char* line, char *filename)
{
	get_one_line(line, filename, 0);
	att0[0] = data_updata0.pitch;
	att0[1] = data_updata0.roll;
	att0[2] = data_updata0.yaw;

	vn0[0] = data_updata0.vx;
	vn0[1] = data_updata0.vy;
	vn0[2] = data_updata0.vz;

	pos0[0] = data_updata0.lat;
	pos0[1] = data_updata0.lon;
	pos0[2] = data_updata0.high;


}

void attitude2Cbp(double *att, double (*Cbp)[3])
{
	double pitch = att[0];
	double roll = att[1];
	double yaw = att[2];

	Cbp[0][0] = cos(roll)*cos(yaw) + sin(roll)*sin(yaw)*sin(pitch);
	Cbp[0][1] = sin(yaw)*cos(pitch);
	Cbp[0][2] = sin(roll)*cos(yaw) - cos(roll)*sin(yaw)*sin(pitch);
	Cbp[1][0] = -cos(roll)*sin(yaw) + sin(roll)*cos(yaw)*sin(pitch);
	Cbp[1][1] = cos(yaw)*cos(pitch);
	Cbp[1][2] = -sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch);
	Cbp[2][0] = -sin(roll)*cos(pitch);
	Cbp[2][1] = sin(pitch);
	Cbp[2][2] = cos(roll)*cos(pitch);
}

double sign(double x)
{
	if (x > 0)
	{
		return 1.0;
	}
	else if (x < 0)
	{
		return -1.0;
	}
	else
	{
		return 0;
	}


}
void Cbp2q(double *q, double(*T)[3])
{
	/*四元数求解，秦P304 选择q0的符号为正*/
	double q1_ab, q2_ab, q3_ab;
	double q2_sign, q1_sign, q3_sign;
	double sign_q0;

	q[0] = fabs(sqrt(fabs(1 + T[0][0] + T[1][1] + T[2][2])) / 2);
	
	sign_q0 = sign(q[0]);
	

	q1_ab = sqrt(fabs(1 + T[0][0] - T[1][1] - T[2][2])) / 2;
	q1_sign = sign_q0*(sign(T[2][1] - T[1][2]));
	q[1] = q1_sign*q1_ab;

	q2_ab = sqrt(fabs(1 - T[0][0] + T[1][1] - T[2][2])) / 2;
	q2_sign = sign_q0*(sign(T[0][2] - T[2][0]));
	q[2] = q2_sign*q2_ab;

	q3_ab = sqrt(fabs(1 - T[0][0] - T[1][1] + T[2][2])) / 2;
	q3_sign = sign_q0 *(sign(T[1][0] - T[0][1]));
	q[3] = q3_sign*q3_ab;
	
}

void q2Cbp(double *q, double(*T)[3])
{
	T[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];

	T[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
	T[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]); 
	T[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
	
	T[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];

	T[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
	T[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
	T[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);

	T[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
		
}
/*
 *	卡尔曼参数初始化
 */
void Q_Init(double(*Q)[15])
{
	Q[3][3] = 2e-04;
	Q[4][4] = 2e-04;
	Q[5][5] = 2e-04;
}
void R_Init(double(*R)[6])
{
	R[0][0] = 0.25;
	R[1][1] = 0.25;
	R[2][2] = 0.25;
	//R[3][3] = 0.00000000000000001;
	//R[4][4] = 0.00000000000000001;
	R[4][4] = 1e-17;
	R[3][3] = 1e-17;
	R[5][5] = 0.001;
}
void P_Init(double(*P)[15])
{
	P[0][0] = 0.000000021153987;
	P[1][1] = 0.000000021153987;
	P[2][2] = 0.000033846379976;
	P[3][3] = 0.25;
	P[4][4] = 0.25;
	P[5][5] = 0.25;
	P[6][6] = 0.000000000000001;
	P[7][7] = 0.000000000000001;
	P[8][8] = 0.01;
}
void H_Init(double(*H)[15]) /*这里为什么是15*/
{
	int i, j;
	H[0][3] = 1.0;
	H[1][4] = 1.0;
	H[2][5] = 1.0;
	H[3][6] = 1.0;
	H[4][7] = 1.0;
	H[5][8] = 1.0;
}

/*
 * 以上是Init的参数,以下是正式的处理函数
 */
/* 四舍五入函数 */
void roundn(double *x, int n)
{

	double a = *x, b;
	int i = 0;
	char stra[100], strb[100];


	a = a * pow((double)10, n);
	sprintf(stra, "%.64lf", a);     //浮点数转换字符串，可以用gcvt转换,gevt取四舍五入 
	while (stra[i] != '.')
	{
		strb[i] = stra[i];
		i++;
	}
	b = atof(strb);        //字符串转换浮点数 

	if (a - b >= 0.5)  b = b + 1;
	b = b*pow((double)10, -n);

	*x = b;
	
}



void Dis31_Dis33Dis31(double(*Dis33)[3], double *Dis31, double *Dis31_result) /* fn = Cnb * fb;*/
{
	Dis31_result[0] = Dis33[0][0] * Dis31[0] + Dis33[0][1] * Dis31[1] + Dis33[0][2] * Dis31[2];
	Dis31_result[1] = Dis33[1][0] * Dis31[0] + Dis33[1][1] * Dis31[1] + Dis33[1][2] * Dis31[2];
	Dis31_result[2] = Dis33[2][0] * Dis31[0] + Dis33[2][1] * Dis31[1] + Dis33[2][2] * Dis31[2];
}

void attitude_rate_cal( double(*Cnb)[3], double *wpbb)
{
	double gyro[3] = { data_updata0.wx, data_updata0.wy, data_updata0.wz };
	double tmp[3] = { eth0.wnie[0] + eth0.wnen[0], eth0.wnie[1] + eth0.wnen[1], eth0.wnie[2] + eth0.wnen[2] };

	wpbb[0] = gyro[0] - Cnb[0][0] * tmp[0] - Cnb[1][0] * tmp[1] - Cnb[2][0] * tmp[2];
	wpbb[1] = gyro[1] - Cnb[0][1] * tmp[0] - Cnb[1][1] * tmp[1] - Cnb[2][1] * tmp[2];
	wpbb[2] = gyro[2] - Cnb[0][2] * tmp[0] - Cnb[1][2] * tmp[1] - Cnb[2][2] * tmp[2];

}
void quat_update_rk4(double *wpbb0, double *wpbb1, double *wpbb2, double nts, double *qnb)
{
	double k1[4], k2[4], k3[4], k4[4], k[4];
	double q1[4];
	double C1[4][4] = {         0, -1 * wpbb0[0], -1 * wpbb0[1], -1 * wpbb0[2],
		                 wpbb0[0],             0,      wpbb0[2], -1 * wpbb0[1],
						 wpbb0[1], -1 * wpbb0[2],             0,      wpbb0[0],
						 wpbb0[2],      wpbb0[1], -1 * wpbb0[0],          0  };

	double C2[4][4] = { 0, -1 * wpbb1[0], -1 * wpbb1[1], -1 * wpbb1[2],
						wpbb1[0], 0, wpbb1[2], -1 * wpbb1[1],
						wpbb1[1], -1 * wpbb1[2], 0, wpbb1[0],
						wpbb1[2], wpbb1[1], -1 * wpbb1[0], 0 };

	double C3[4][4] = { 0, -1 * wpbb2[0], -1 * wpbb2[1], -1 * wpbb2[2],
						wpbb2[0], 0, wpbb2[2], -1 * wpbb2[1],
						wpbb2[1], -1 * wpbb2[2], 0, wpbb2[0],
						wpbb2[2], wpbb2[1], -1 * wpbb2[0], 0 };

	/*龙格库塔法*/
	/* k1 = 0.5 * C1 * qnb ; */
	k1[0] = 0.5 * (C1[0][0] * qnb[0] + C1[0][1] * qnb[1] + C1[0][2] * qnb[2] + C1[0][3] * qnb[3]);
	k1[1] = 0.5 * (C1[1][0] * qnb[0] + C1[1][1] * qnb[1] + C1[1][2] * qnb[2] + C1[1][3] * qnb[3]);
	k1[2] = 0.5 * (C1[2][0] * qnb[0] + C1[2][1] * qnb[1] + C1[2][2] * qnb[2] + C1[2][3] * qnb[3]);
	k1[3] = 0.5 * (C1[3][0] * qnb[0] + C1[3][1] * qnb[1] + C1[3][2] * qnb[2] + C1[3][3] * qnb[3]);



	/* q1 = k1*h_in/2+Q_in; */																
	q1[0] = k1[0] * nts  * 0.5 + qnb[0];
	q1[1] = k1[1] * nts  * 0.5 + qnb[1];
	q1[2] = k1[2] * nts  * 0.5 + qnb[2];
	q1[3] = k1[3] * nts  * 0.5 + qnb[3];

	/* k2 = 0.5 * C2 * q1; */
	k2[0] = 0.5 * (C2[0][0] * q1[0] + C2[0][1] * q1[1] + C2[0][2] * q1[2] + C2[0][3] * q1[3]);
	k2[1] = 0.5 * (C2[1][0] * q1[0] + C2[1][1] * q1[1] + C2[1][2] * q1[2] + C2[1][3] * q1[3]);
	k2[2] = 0.5 * (C2[2][0] * q1[0] + C2[2][1] * q1[1] + C2[2][2] * q1[2] + C2[2][3] * q1[3]);
	k2[3] = 0.5 * (C2[3][0] * q1[0] + C2[3][1] * q1[1] + C2[3][2] * q1[2] + C2[3][3] * q1[3]);

	/* q2 = k2*h_in/2+Q_in; */
	q1[0] = k2[0] * nts  * 0.5 + qnb[0];
	q1[1] = k2[1] * nts  * 0.5 + qnb[1];
	q1[2] = k2[2] * nts  * 0.5 + qnb[2];
	q1[3] = k2[3] * nts  * 0.5 + qnb[3];

	/* k3 = 0.5 * C2 * q2; */
	k3[0] = 0.5 * (C2[0][0] * q1[0] + C2[0][1] * q1[1] + C2[0][2] * q1[2] + C2[0][3] * q1[3]);
	k3[1] = 0.5 * (C2[1][0] * q1[0] + C2[1][1] * q1[1] + C2[1][2] * q1[2] + C2[1][3] * q1[3]);
	k3[2] = 0.5 * (C2[2][0] * q1[0] + C2[2][1] * q1[1] + C2[2][2] * q1[2] + C2[2][3] * q1[3]);
	k3[3] = 0.5 * (C2[3][0] * q1[0] + C2[3][1] * q1[1] + C2[3][2] * q1[2] + C2[3][3] * q1[3]);

	/* q3 = k3*h_in+Q_in; */
	q1[0] = k3[0] * nts  * 0.5 + qnb[0];
	q1[1] = k3[1] * nts  * 0.5 + qnb[1];
	q1[2] = k3[2] * nts  * 0.5 + qnb[2];
	q1[3] = k3[3] * nts  * 0.5 + qnb[3];

	/* k4 = 0.5 * C3 * q3; */
	k4[0] = 0.5 * (C3[0][0] * q1[0] + C3[0][1] * q1[1] + C3[0][2] * q1[2] + C3[0][3] * q1[3]);
	k4[1] = 0.5 * (C3[1][0] * q1[0] + C3[1][1] * q1[1] + C3[1][2] * q1[2] + C3[1][3] * q1[3]);
	k4[2] = 0.5 * (C3[2][0] * q1[0] + C3[2][1] * q1[1] + C3[2][2] * q1[2] + C3[2][3] * q1[3]);
	k4[3] = 0.5 * (C3[3][0] * q1[0] + C3[3][1] * q1[1] + C3[3][2] * q1[2] + C3[3][3] * q1[3]);

	k[0] = (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6;
	k[1] = (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6;
	k[2] = (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6;
	k[3] = (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]) / 6;


	qnb[0] = qnb[0] + k[0] * nts;
	qnb[1] = qnb[1] + k[1] * nts;
	qnb[2] = qnb[2] + k[2] * nts;
	qnb[3] = qnb[3] + k[3] * nts;


}

void qNormalize(double *q)
{
	double tmp = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

	q[0] /= tmp;
	q[1] /= tmp;
	q[2] /= tmp;
	q[3] /= tmp;

}
void Dis33_Dis33Dis33(double(*left)[3], double(*right)[3], double(*result)[3])
{
	result[0][0] = left[0][0] * right[0][0] + left[0][1] * right[1][0] + left[0][2] * right[2][0];
	result[0][1] = left[0][0] * right[0][1] + left[0][1] * right[1][1] + left[0][2] * right[2][1];
	result[0][2] = left[0][0] * right[0][2] + left[0][1] * right[1][2] + left[0][2] * right[2][2];
	
	result[1][0] = left[1][0] * right[0][0] + left[1][1] * right[1][0] + left[1][2] * right[2][0];
	result[1][1] = left[1][0] * right[0][1] + left[1][1] * right[1][1] + left[1][2] * right[2][1];
	result[1][2] = left[1][0] * right[0][2] + left[1][1] * right[1][2] + left[1][2] * right[2][2];
	
	result[2][0] = left[2][0] * right[0][0] + left[2][1] * right[1][0] + left[2][2] * right[2][0];
	result[2][1] = left[2][0] * right[0][1] + left[2][1] * right[1][1] + left[2][2] * right[2][1];
	result[2][2] = left[2][0] * right[0][2] + left[2][1] * right[1][2] + left[2][2] * right[2][2];

}


void state_matrix_assignment(double *vn, double *fn, double(*Mpv)[3], double(*Cnb)[3], float nts, double(*F)[15])
{
	int i = 0, j = 0;     /*15*15矩阵*/
	int i_small, j_small; /*3*3矩阵*/
	double f_RNh = 1.562822160083740e-07, f_RMh = 1.563336726765474e-07, t1 = -4.412311342183346;
	double g0 = 9.7803267714;  
	double scl = eth0.sl*eth0.cl;
	double vE_clRNh = vn[0] * ( -7.071288247111441e-07 );
	double Avn[3][3] = { 0, -1 * vn[2], vn[1], vn[2], 0, -1 * vn[0], -1 * vn[1], vn[0], 0 }; /*纯速度项*/
	double Awn[3][3] = { 0, -1 * eth0.wnien[2], eth0.wnien[1], eth0.wnien[2], 0, -1 * eth0.wnien[0], -1 * eth0.wnien[1], eth0.wnien[0], 0 };

	double Maa[3][3] = { 0, eth0.wnin[2], -1 * eth0.wnin[1], -1 * eth0.wnin[2], 0, eth0.wnin[0], eth0.wnin[1], -1 * eth0.wnin[0], 0 };
	double Mav[3][3] = { 0, -1 * f_RMh, 0, f_RNh, 0, 0, f_RNh * eth0.tl, 0, 0 };
	double Mva[3][3] = { 0, -1 * fn[2], fn[1], fn[2], 0, -1 * fn[0], -1 * fn[1], fn[0], 0 };

	/* Mvv = Avn*Mav - Awn;*/
	double Mvv[3][3], Mvp[3][3] = { 0 };
	Dis33_Dis33Dis33(Avn ,Mav,Mvv);
	
	Mvp[2][0] = Mvp[2][0] - g0*(5.27094e-3 * 2 * scl + 2.32718e-5 * 4 * eth0.sl2*scl);
	Mvp[2][2] = Mvp[2][2] + 3.086e-6;

	double Mpp[3][3] = { 0, 0, 0, vE_clRNh*t1, 0, 0, 0, 0, 0 };
	double Ft[15][15] = { 0 };



	/*Ft赋值,写的太啰嗦了....*/
	for (i = 0, i_small = 0; i < 3; i++, i_small++)
	{
		for (j = 0, j_small=0; j < 3; j++, j_small++)
		{
			Ft[i][j] = Maa[i_small][j_small] * nts;
		}
	}
	for (i = 0, i_small = 0; i < 3; i++, i_small++)
	{
		for (j = 3, j_small = 0; j < 6; j++, j_small++)
		{
			Ft[i][j] = Mav[i_small][j_small] * nts;
		}
	}
	for (i = 3, i_small = 0; i < 6; i++, i_small++)
	{
		for (j = 0, j_small = 0; j < 3; j++, j_small++)
		{
			Ft[i][j] = Mva[i_small][j_small] * nts;
		}
	}

	for (i = 3, i_small = 0; i < 6; i++, i_small++)
	{
		for (j = 3, j_small = 0; j < 6; j++, j_small++)
		{
			Ft[i][j] = Mvv[i_small][j_small] * nts;
		}
	}
	for (i = 6, i_small = 0; i < 9; i++, i_small++)
	{
		for (j = 3, j_small = 0; j < 6; j++, j_small++)
		{
			Ft[i][j] = Mpv[i_small][j_small] * nts;
		}
	}
	for (i = 3, i_small = 0; i < 6; i++, i_small++)
	{
		for (j = 6, j_small = 0; j < 9; j++, j_small++)
		{
			Ft[i][j] = Mvp[i_small][j_small] * nts;
		}
	}
	for (i = 6, i_small = 0; i < 9; i++, i_small++)
	{
		for (j = 6, j_small = 0; j < 9; j++, j_small++)
		{
			Ft[i][j] = Mpp[i_small][j_small] * nts;
		}
	}
	for (i = 0, i_small = 0; i < 3; i++, i_small++)
	{
		for (j = 9, j_small = 0; j < 12; j++, j_small++)
		{
			Ft[i][j] = -1 * Cnb[i_small][j_small] * nts;
		}
	}
	for (i = 3, i_small = 0; i < 6; i++, i_small++)
	{
		for (j = 12, j_small = 0; j < 15; j++, j_small++)
		{
			Ft[i][j] = Cnb[i_small][j_small] * nts;
		}
	}
	
	/*Fk = eye(size(Ft)) + Fk;*/
	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 15; j++)
		{
			F[i][j] = Ft[i][j];
			if (i == j)
			{
				F[i][j] += 1;
			}
			
		}

	}
}

/*Kalman 公式1	xk = F * xk; */
void Kalman_Formula_1(double *xk, double(*Mpv)[15])
{
	int i = 0, j = 0;
	double xk_new[15] = {0};
	double tmp = 0.0;
	
	for (i = 0; i < 15; i++)
	{
		tmp = 0.0;
		for (j = 0; j < 15; j++)
		{			
			tmp = tmp + ( Mpv[i][j] * xk[j] );
		}
		xk_new[i] = tmp;
	}
	

	for (i = 0; i < 15; i++)
	{
		xk[i] = xk_new[i];
	}


}

/*Kalman 公式2   P = F * P * F' + Q;*/
void Kalman_Formula_2(double(*P)[15], double(*F)[15], double(*Q)[15])
{

	double P_new[15][15] = { 0 };
	//double P_new2[15][15] = { 0 };
	int i = 0, j = 0, k = 0;
	double tmp = 0.0;

	/*P_New = F * P*/
	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 15 ; j++)
		{
			for (k = 0; k < 15; k++)
			{
				tmp += F[i][k] * P[k][j];
			}
	
			P_new[i][j] = tmp;
			tmp = 0;

			
		}
	}

	/* F*P*F' + Q */
	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 15; j++)
		{
			for (k = 0; k < 15; k++)
			{
				tmp += P_new[i][k] * F[j][k];
			}

			P[i][j] = tmp + Q[i][j];
			tmp = 0;


		}
	}

}

/*
 * 融合进GPS参数的if操作
 */
void vnGPS_LoadData(double *vnGPS)
{
	vnGPS[0] = data_updata0.vx;
	vnGPS[1] = data_updata0.vy;
	vnGPS[2] = data_updata0.vz;
}
void posGPS_LoadData(double *posGPS)
{
	posGPS[0] = data_updata0.lat;
	posGPS[1] = data_updata0.lon;
	posGPS[2] = data_updata0.high;
}

void yk_LoadData(double *yk, double *vnGPS,double *posGPS,double *vn,double *pos)
{
	int i = 0;
	for (i = 0; i < 3; i++)
	{
		yk[i] = vn[i] - vnGPS[i];
		yk[i + 3] = pos[i] - posGPS[i];
	}
}

long double getA(long double(*arcs)[6], int n)//按第一行展开计算|A|
{
	if (n == 1)
	{
		return arcs[0][0];
	}
	long double ans = 0;
	long double temp[6][6];
	int i, j, k;
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n - 1; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];

			}
		}
		long double t = getA(temp, n - 1);
		if (i % 2 == 0)
		{
			ans += arcs[0][i] * t;
		}
		else
		{
			ans -= arcs[0][i] * t;
		}
	}
	

	return ans;
}
/* 直接求逆即可 ,是个6*6的矩阵*/
void getAStart(long double(*arcs)[6], long double(*ans)[6])//计算每一行每一列的每个元素所对应的余子式，组成A*
{
	int n = 6;
	int i, j, k, t;
	long double temp[6][6];
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				for (t = 0; t<n - 1; t++)
				{
					temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
				}
			}


			ans[j][i] = getA(temp, n - 1);
			
			if ((i + j) % 2 == 1)
			{
				ans[j][i] = -ans[j][i];
			}
		}
	}
}

int MatrixInv(int n, double ra[], double rb[])
{
	//int i, j, k, l, u, v, is[10], js[10];   /* matrix dimension <= 10 */
	int i, j, k, l, u, v;
	int *is = (int*)malloc(sizeof(int)*n);
	int *js = (int*)malloc(sizeof(int)*n);
	double d, p;

	if (n <= 0)
	{
		printf("Error dimension in MatrixInv!\n");
		exit(EXIT_FAILURE);
	}

	/* 将输入矩阵赋值给输出矩阵rb，下面对rb矩阵求逆，a矩阵不变 */
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			rb[i*n + j] = ra[i*n + j];
		}
	}

	for (k = 0; k<n; k++)
	{
		d = 0.0;
		for (i = k; i<n; i++)   /* 查找右下角方阵中主元素的位置 */
		{
			for (j = k; j<n; j++)
			{
				l = n*i + j;
				p = fabs(rb[l]);
				if (p>d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (d<DBL_EPSILON)   /* 主元素接近于0，矩阵不可逆 */
		{
			printf("Divided by 0 in MatrixInv!\n");
			return 0;
		}

		if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
		{
			for (j = 0; j<n; j++)
			{
				u = k*n + j;
				v = is[k] * n + j;
				p = rb[u];
				rb[u] = rb[v];
				rb[v] = p;
			}
		}

		if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
		{
			for (i = 0; i<n; i++)
			{
				u = i*n + k;
				v = i*n + js[k];
				p = rb[u];
				rb[u] = rb[v];
				rb[v] = p;
			}
		}

		l = k*n + k;
		rb[l] = 1.0 / rb[l];  /* 初等行变换 */
		for (j = 0; j<n; j++)
		{
			if (j != k)
			{
				u = k*n + j;
				rb[u] = rb[u] * rb[l];
			}
		}
		for (i = 0; i<n; i++)
		{
			if (i != k)
			{
				for (j = 0; j<n; j++)
				{
					if (j != k)
					{
						u = i*n + j;
						rb[u] = rb[u] - rb[i*n + k] * rb[k*n + j];
					}
				}
			}
		}
		for (i = 0; i<n; i++)
		{
			if (i != k)
			{
				u = i*n + k;
				rb[u] = -rb[u] * rb[l];
			}
		}
	}

	for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
	{
		if (js[k] != k)
		{
			for (j = 0; j<n; j++)
			{
				u = k*n + j;
				v = js[k] * n + j;
				p = rb[u];
				rb[u] = rb[v];
				rb[v] = p;
			}
		}
		if (is[k] != k)
		{
			for (i = 0; i<n; i++)
			{
				u = i*n + k;
				v = is[k] + i*n;
				p = rb[u];
				rb[u] = rb[v];
				rb[v] = p;
			}
		}
	}
	free(is);
	free(js);
	return (1);
}






/* K = P * H' * inv(H * P * H' + R); */
void Kalman_Formula_3(double(*P)[15], double(*H)[15], double(*R)[6], double(*K)[6]) 
{
	int i = 0, j = 0, k = 0;

	double x0[36] = { 0 }, result[36] = { 0 };
	double H_trans[15][6];
	double P_Htrans[15][6] = {0};/*P_Htrans =  P * H'*/
	double ans_HP[6][15] = { 0 };
	double ans_HPHt[6][6] = { 0 };
	long double HPHtrans_R[6][6] = { 0 }; /*求逆之前的最终矩阵 6*6 */

	long double ans_A ;       /*矩阵的行列式*/
	double ans[6][6] = { 0 }; /*求逆后的结果*/


	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 6; j++)
		{
			H_trans[i][j] = H[j][i];
		}
	}



	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 6; j++)
		{
			for (k = 0; k < 15; k++)
			{
				P_Htrans[i][j] += P[i][k] * H_trans[k][j];
				
			}
			K[i][j] = P_Htrans[i][j];
		}
	
	}


	/*矩阵求逆的过程*/
	/* H * P * H' + R  (6*15) (15*15) (15*6) + (6*6) */
	/* 1) ans_HP = H * P   */
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 15; j++)
		{
			for (k = 0; k < 15; k++)
			{
				ans_HP[i][j] += H[i][k] * P[k][j];
			}
		}
	}


	/* 2) ans_HPHt = ans_HP * H'  */
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			for (k = 0; k < 15; k++)
			{
				ans_HPHt[i][j] += ans_HP[i][k] * H_trans[k][j];
			}
		}
	}
	/* 3) HPHtrans_R = ans_HPHt + R   */
	k = 0;
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			HPHtrans_R[i][j] = (long double)(ans_HPHt[i][j] + R[i][j]);
			x0[k++] = HPHtrans_R[i][j];
		}
		
	}

	//printf("HPHtrans_R\r\n");
	//for (i = 0; i < 6; i++)
	//{
	//	for (j = 0; j < 6; j++)
	//	{
	//		printf("%.10f\r\n", HPHtrans_R[i][j]);
	//	}
	//	printf("=======================\r\n");
	//}
	//getchar();






	k = 0;
	MatrixInv(6,x0,result);

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			ans[i][j] = result[k++];
		}
	}

	//printf("tmp_inv\r\n");
	//for (i = 0; i < 6; i++)
	//{
	//	for (j = 0; j < 6; j++)
	//	{
	//		printf("%.10f\r\n", ans[i][j]);
	//	}
	//	printf("=======================\r\n");
	//}
	//getchar();



	for (i = 0; i < 15; i++)
	{

		for (j = 0; j < 6; j++)
		{
			for (k = 0; k < 6; k++)
			{
				K[i][j] += P_Htrans[i][k] * ans[k][j];
			}
		}

	}



#if 0
	ans_A = getA(HPHtrans_R, 6); /*这个ans算的就是有问题的,但是还是不知道为什么*/

	if (ans_A == 0.0)
	{
		printf("ans_A Error");
	}	


	getAStart(HPHtrans_R, ans);


	/* inv(H * P * H' + R) 矩阵误差已经很大了 */
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			ans[i][j] = ans[i][j] / ans_A;
		}
	}

	printf("K = PHans\r\n");
	for (i = 0; i < 15;i++)
	{
		
		for (j = 0; j < 6; j++)
		{
			for (k = 0; k < 6; k++)
			{
				K[i][j] += P_Htrans[i][k] * ans[k][j];
			}
		}

	}

#endif

}
/* xk = xk + K * (yk - H * xk); */
void Kalman_Formula_4(double(*K)[6],double *yk, double (*H)[15], double *xk)
{
	int i, j, k;
	double Hxk[6] = { 0 };
	double yk_Hxk[6] = { 0 };
	double K_yk_Hxk[15] = {0};/* K * (yk - H * xk)*/



	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 15; j++)
		{
			Hxk[i] += H[i][j] * xk[j];			
		}
		yk_Hxk[i] =yk[i] -  Hxk[i];
		
		
	}


	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 6; j++)
		{
			K_yk_Hxk[i] += K[i][j] * yk_Hxk[j];
		}
		xk[i] += K_yk_Hxk[i];

	}



}

/* P = P - K * (H * P * H' + R) * K'; */
void Kalman_Formula_5(double(*K)[6], double(*H)[15], double(*R)[6], double(*P)[15])
{
	double H_tran[15][6] = { 0 };
	double K_tran[6][15] = { 0 };
	double HP[6][15] = { 0 };
	double HP_Htran_R[6][6] = { 0 };
	double K_HPHtran_R[15][6] = { 0 };
	double KHPHtran_R_Ktran[15][15] = { 0 };
	int i, j, k;
	/* 转置矩阵初始化 */


	for (i = 0; i <15 ; i++)
	{
		for (j = 0; j <6 ; j++)
		{
			H_tran[i][j] = H[j][i];
		}		
	}

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 15; j++)
		{
			K_tran[i][j] = K[j][i];
		}
	}
	


	/* H * P */
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 15; j++)
		{
			for (k = 0; k < 15; k++)
			{
				HP[i][j] += H[i][k] * P[k][j];
			}
		}
	}

	/* H*P * H' + R */
	for (i = 0; i < 6; i++)
	{
		
		for (j = 0; j < 6; j++)
		{
			for (k = 0; k < 15; k++)
			{
				HP_Htran_R[i][j] += HP[i][k] * H_tran[k][j];
			}
			HP_Htran_R[i][j] = HP_Htran_R[i][j] + R[i][j];
		}
	}

	/* K * (H*P * H' + R) */
	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 6; j++)
		{
			for (k = 0; k < 6; k++)
			{

				K_HPHtran_R[i][j] += K[i][k] * HP_Htran_R[k][j];
			}
		}
		
	}

	/* [K * (H*P * H' + R)] *K'  */
	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 15; j++)
		{
			for (k = 0; k < 6; k++)
			{
				KHPHtran_R_Ktran[i][j] += K_HPHtran_R[i][k] * K_tran[k][j];
			}
			P[i][j] -= KHPHtran_R_Ktran[i][j];	
		}
	}
}


void Cpc_LoadData(double *xk, double(*Cpc)[3])
{
	Cpc[0][0] = 1;
	Cpc[1][1] = 1;
	Cpc[2][2] = 1;
	Cpc[0][1] = xk[2];
	Cpc[0][2] = -1 * xk[1];
	Cpc[1][0] = -1 * xk[2];
	Cpc[1][2] = xk[0];
	Cpc[2][0] = xk[1];
	Cpc[2][1] = -1 * xk[0];
}

void Cnb_CpctranCnb(double(*Cpc)[3], double(*Cnb)[3])
{
	int i = 0, j = 0, k = 0;
	double Cpc_tran[3][3] = {0};
	double Cnb_shadow[3][3] = { 0 };
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			Cnb_shadow[i][j] = Cnb[i][j];
			Cnb[i][j] = 0;
		}
	}

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			Cpc_tran[i][j] = Cpc[j][i];
		}
	}

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			for (k = 0; k < 3; k++)
			{
				Cnb[i][j] += Cpc_tran[i][k] * Cnb_shadow[k][j];
			}
		}
	}


}
void vn_pos_xk(double *vn ,double *pos,double *xk)
{
	int i, j;
	for (i = 0; i < 3; i++)
	{
		vn[i]  -= xk[i + 3];
		pos[i] -= xk[i + 6];
	}
}

void xk_Init(double *xk)
{
	int i = 0;
	for (i = 0; i < 15; i++)
	{
		xk[i] = 0;
	}
}

/* 
 * attitude=Cnb2attitude(T)  求解姿态角 依据秦297的表格
 */
void Cnb2attitude(double (*Cnb)[3], double *att)
{
	int i, j, k;
	double pitch = 0, roll = 0, yaw = 0;

	pitch = asin(Cnb[2][1]);
	
	roll = atan(-Cnb[2][0] / Cnb[2][2]);

	if ((Cnb[2][2]<0) && (roll>0))
	{
		roll = roll - pi;
	}
	else if ((Cnb[2][2] < 0) && (roll < 0))
	{
		roll += pi;
	}

	yaw = atan(Cnb[0][1] / Cnb[1][1]);

	if (fabs(Cnb[1][1]) == 0)
	{

		if (Cnb[0][1]>0)
			yaw = pi / 2;
		else
			yaw = -1 * pi / 2;
	}
	else if (Cnb[1][1] < 0)
	{
		if (Cnb[0][1] > 0)
		{
			yaw += pi;
		}
		else
		{
			yaw -= pi;
		}
			
	}


	att[0] = pitch;
	att[1] = roll;
	att[2] = yaw;
	


}

int main()
{
	FILE *fp;
	char ch[500] ;
	fp = fopen("C:\\Users\\Administrator\\Desktop\\data_C.txt", "w"); /*输入数据地址*/
	fclose(fp);//这样就清空了内容，简单！

	int i, j, k;
	rad2deg = 180 / pi;			//弧度变角度
	deg2rad = pi / 180;			//角度变弧度
	longitude = longitude *deg2rad;
	latitude  = latitude  * deg2rad; // 变成弧度

	//printf(" pi=%f, g=%f, phi=%f, lamda=%f, Re=%f, deg2rad=%f, rad2deg=%f ", pi, g, phi, lamda, Re, deg2rad, rad2deg );

	//char filename[] = "C:\\Users\\Administrator\\Desktop\\data_csv.csv";
	char filename[] = "data_csv.csv";
	char line[1024];
	double **data;
	int row, col; //行 列
	int row_now = 0, col_now = 0;

	/*
	 *Matlab for循环里面的参数
     */
	int longeT=0; /*因为龙格库塔是一次跳三行*/
	int k1 = 1;
	double T;
	double fb[3];
	double fn[3];   /* fn = Cnb * fb */
	double vn1[3];
	double t_small = 0.0; /* Matlab上的 T 就是 t_small */

	/*
	 * if里的参数
	 */
	/* modf的参数 */
	double  t_small_decimal; /* Matlab中的T转换成小数 */
	double *t_small_integer = NULL; /* Matlab中的T转换成整数 */

	double vnGPS[3];
	double posGPS[3];
	double yk[6];


	


	/*位置更新*/
	double Mpvvn[3];

	/*姿态角更新*/
	double wpbb0[3];
	double wpbb1[3];
	double wpbb2[3];

	/*Kalman参数*/
	double F[15][15] = { 0 };

	double 	B[15][15] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

	double A[15] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
	
	double Cpc[3][3] = { 0 };

	double att[3] = { 0 };

	int ki = 0; /* 用于 result的行数  */



	/*
	 * 初始化部分
	 */	 

	row = get_row(filename);
	col = get_col(filename);
	
	data = (double **)malloc(row * sizeof(int *));
	for (i = 0; i < row; ++i)
	{
		data[i] = (double *)malloc(col * sizeof(double));	
	}
	printf("data load over\r\n");

	fp = fopen("1.txt", "w");//假如打开的是1.txt文件
	fclose(fp);//这样就清空了内容，简单！
	fp = fopen("C:\\Users\\Administrator\\Desktop\\data_C.txt", "r+");

	parameter_init(line,filename);
	printf("Init over\r\n");

	attitude2Cbp(att0, Cnb0);
	printf("Attitude2Cbp over\r\n");

	Cbp2q(qnb0, Cnb0);
	printf("Q_nb over\r\n");

	q2Cbp(qnb0, Cnb0);
	printf("Cnb0 over\r\n");

	/*
	 * Cnb  = Cnb0;
	 */
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			Cnb[i][j] = Cnb0[i][j];
		}
	}
	/*
	 * vn = vn0;  pos = pos0;
	 */
	for (j = 0; j < 3; j++)
	{
		vn[j] = vn0[j];	
		pos[j] = pos0[j];
	}
	/* qnb = qnb0; */
	for (j = 0; j < 4; j++)
	{
		qnb[j] = qnb0[j];
	}


	/*
	* Kalman 滤波的参数调教
	*/
	Q_Init(Q);
	R_Init(R);


	H_Init(H);
	P_Init(P);
	printf("Kalman Init over\r\n");



	printf("Wait\r\n");

	for (row_now = 0, col_now = 0; row_now < row; row_now += 2)
	{
		k1 += 2;
		get_one_line(line, filename, k1-2);
		
		t_small = data_updata0.minute - 7 + data_updata0.second;

		/* 四舍五入函数 ,Matlab上的 T 就是 t_small */
		roundn( &t_small, 2);   /*这个暂时有问题,最后是10.300000000000001 这样的,但是和Matlab上是一样的*/									  

		//printf("t_small : %.20f\r\n", t_small);
		/*
		 * 速度更新
		 */
		get_one_line(line, filename, row_now);
		fb[0] = data_updata0.ax;
		fb[1] = data_updata0.ay;
		fb[2] = data_updata0.az;

		Dis31_Dis33Dis31(Cnb, fb, fn); /* 3*1 = 3*3 × 3*1 ; fn = Cnb * fb */
		
		vn[0] = fn[0] * nts + vn[0];
		vn[1] = fn[1] * nts + vn[1];
		vn[2] = fn[2] * nts + vn[2];

		/*
		 * 位置更新
		 */
		Dis31_Dis33Dis31(Mpv, vn, Mpvvn);/*3*1 = 3*3 × 3*1 ; Mpvvn = Mpv*vn1;*/
		pos[0] = Mpvvn[0] * nts + pos[0];
		pos[1] = Mpvvn[1] * nts + pos[1];
		pos[2] = Mpvvn[2] * nts + pos[2];

		/*
		 * 姿态角更新,一次取得三组惯导数据
		 */
		
		get_one_line(line, filename, longeT);
		attitude_rate_cal( Cnb, wpbb0);
		get_one_line(line, filename, longeT++);
		attitude_rate_cal( Cnb, wpbb1);
		get_one_line(line, filename, longeT++);
		attitude_rate_cal( Cnb, wpbb2);

		quat_update_rk4(wpbb0, wpbb1, wpbb2, nts, qnb);
		qNormalize(qnb);
		q2Cbp(qnb, Cnb);

		state_matrix_assignment(vn, fn, Mpv, Cnb,nts , F);


		/*Kalman 公式1	xk = F * xk; */
		Kalman_Formula_1(xk,F);
		

		/*Kalman 公式2   P = F * P * F' + Q;*/
		Kalman_Formula_2(P ,F , Q);
		
		/*处理Matlab的T ,因为C语言中没有mod函数*/
		t_small_decimal = modf(t_small, &t_small_integer);  //i中存放整数部分,f中存放小数部分

		//for (i = 0; i <15; i++)
		//{
		//	for (j = 0; j <15; j++)
		//	{
		//		printf("%.15f\r\n",P[i][j]);
		//	}
		//	printf("-------------------------\r\n");
		//}
		//getchar();


		/* 融入GPS数据,if内部循环 ,此时P不能作为参考,因为if内部有改变P矩阵*/
		if (t_small_decimal <ts)
		{
			/* vnGPS = AVP(k1,4:6)' ;*/
			get_one_line(line, filename, k1);
			vnGPS_LoadData(vnGPS);
			posGPS_LoadData(posGPS);

			yk_LoadData(yk,vnGPS,posGPS,vn,pos);



	
						
			/*公式4 :  K = P * H' * inv(H * P * H' + R);*/
			Kalman_Formula_3(P , H , R , K);/*K值虽然不太对,但是应该是正确的值*/
			
			/*公式5:xk = xk + K * (yk - H * xk);*/
			Kalman_Formula_4(K, yk, H, xk);



			/*公式6: P = P - K * (H * P * H' + R) * K';   */
			Kalman_Formula_5( K, H, R, P);

			q2Cbp(qnb,Cnb);
		
			Cpc_LoadData(xk,Cpc);/* Cpc 数据已经不太对了 */

			/* Cnb= Cpc' * Cnb; */
			Cnb_CpctranCnb(Cpc,Cnb);

			/*  qnb = Cbp2q(Cnb); */
			Cbp2q(qnb, Cnb);


			/* vn = vn - xk(4:6); pos = pos - xk(7:9);*/
			vn_pos_xk(vn,pos,xk); /*因为vn与xk进行相减,xk中部分参数与matlab上跑出来的不一样*/

			/*xk(1:15) = zeros(1,15);    %对应xk已补偿部分置零*/
			xk_Init(xk);

			/* Cnb = q2Cbp(qnb); */
			q2Cbp(qnb, Cnb);

			/* att = Cnb2attitude(Cnb); 这个att没有用到过*/
			//Cnb2attitude(Cnb,att);


			printf("vn pos: %.15f,%.15f,%.15f,%.15f,%.15f,%.15f\r\n",vn[0],vn[1],vn[2],pos[0],pos[1],pos[2]);
		
			

			printf("wait a moment %d\r\n",cnt++);


			/* 写入txt文件中 */
			sprintf(ch, "%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.3f", att[0], att[1], att[2], vn[0], vn[1], vn[2], pos[0], pos[1], pos[2], t_small);
			fputs(ch, fp);
			fputs("\n", fp);

			
		}

		//sprintf(ch, "%d", row_now);

		//fputs(ch, fp);
		//fputs("\n",fp);
	}

	fclose(fp);

	printf("\r\nHello World...");
	getchar();
	return 0;



}































/*
//get_one_line(line,filename, 3);
//printf("minute: %d,second :%f\r\n", data_updata0.minute, data_updata0.second);



//get_two_dimension(line, data, filename);

//printf("size : %d",sizeof(data));
//print_two_dimension(data, row, col);




*/
