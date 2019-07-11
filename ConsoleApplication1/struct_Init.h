#pragma once
#ifndef STRUCT_INIT_H
#define STRUCT_INIT_H

struct eth
{
	double e2;
	double wie;
	double sl;
	double cl;
	double tl;
	double sl2;
	double RNh;
	double clRNh;
	double RMh;
	double wnie[3];
	double wnen[3];
	double wnien[3];
	double wnin[3];
};

struct data_updata
{
	int minute;
	double second;
	double ax;
	double ay;
	double az;
	double wx;
	double wy;
	double wz;


	double pitch;
	double roll;
	double yaw;

	double mx;
	double my;
	double mz;

	double pressure;
	int satellite_flag;
	double hight;
	double high;
	double lat;
	double lon;

	double vx;
	double vy;
	double vz;
	double heading;
	int UTC;
};





#endif