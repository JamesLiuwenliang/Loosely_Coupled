#pragma warning(disable: 4996)
#define _CRT_SECURE_NO_WARNINGS
#include<stdio.h>
#include <stdlib.h>
#include <string.h>
#include "struct_Init.h"
#include "csv_operation.h"

extern struct eth eth0;
extern struct data_updata data_updata0;
int get_row(char *filename)
{
	char line[1024];
	int i = 0;
	FILE* stream = fopen(filename, "r");
	while (fgets(line, 1024, stream)){
		i++;
	}
	fclose(stream);
	return i;
}


int get_col(char *filename)
{
	char line[1024];
	int i = 0;
	FILE* stream = fopen(filename, "r");
	fgets(line, 1024, stream);
	char* token = strtok(line, ",");
	while (token)
	{
		token = strtok(NULL, ",");
		i++;
	}
	fclose(stream);
	return i;
}

void get_two_dimension(char* line, double** data, char *filename)
{
	FILE* stream = fopen(filename, "r");
	int i = 0;
	while (fgets(line, 1024, stream))//逐行读取
	{
		int j = 0;
		char *tok;
		char* tmp = strdup(line);
		for (tok = strtok(line, ","); tok && *tok; j++, tok = strtok(NULL, ",\n"))
		{
			data[i][j] = atof(tok);//转换成浮点数 
		}//字符串拆分操作 
		i++;
		free(tmp);
	}
	fclose(stream);//文件打开后要进行关闭操作
}

/*
 *	获取一行的数据,转到结构体中
 */

void get_one_line(char* line,  char *filename, int row_now)
{
	FILE* stream = fopen(filename, "r");
	int i = 0,j = 0;
	char flag = 0;
	char *tok;
	char* tmp = NULL;
	//

	/*要先跳过之前的行数*/
	for (j = 0; j < row_now; j++)
	{
		fscanf(stream, "%*[^\n]%*c");
		/******************************************************
		%*				:	是“跳过”
		[^\n]	        :	字符串的分隔符是"\n", 中括号里可以写 分隔符 表
		%*[^\n]         :	跳过 \n 前的所有字符串。
		%*c 	        :	是“跳过”行尾 的 换行符
		******************************************************/
	}
	
	fgets(line, 1024, stream);
	
	tmp = strdup(line);
	for (tok = strtok(line, ",") ,j =0; tok && *tok; j++, tok = strtok(NULL, ",\n"))
	{
		switch (j)
		{
		case 0:		data_updata0.minute = atoi(tok); break;
		case 1:		data_updata0.second = atof(tok); break;

		case 2:		data_updata0.ax = atof(tok); break;
		case 3:		data_updata0.ay = atof(tok); break;
		case 4:		data_updata0.az = atof(tok); break;

		case 5:		data_updata0.wx = atof(tok); break;
		case 6:		data_updata0.wy = atof(tok); break;
		case 7:		data_updata0.wz = atof(tok); break;

		case 8:		data_updata0.pitch = atof(tok); break;
		case 9:		data_updata0.roll = atof(tok); break;
		case 10:	data_updata0.yaw = atof(tok); break;

		case 11:	data_updata0.mx = atof(tok); break;
		case 12:	data_updata0.my = atof(tok); break;
		case 13:	data_updata0.mz = atof(tok); break;

		case 14:	data_updata0.pressure = atof(tok); break;
		case 15:	data_updata0.satellite_flag = atoi(tok); break;
		case 16:	data_updata0.hight = atof(tok); break;
		case 17:	data_updata0.high = atof(tok); break;
		case 18:	data_updata0.lat = atof(tok); break;
		case 19:	data_updata0.lon = atof(tok); break;
		case 20:	data_updata0.vx = atof(tok); break;
		case 21:	data_updata0.vy = atof(tok); break;
		case 22:	data_updata0.vz = atof(tok); break;
		case 23:	data_updata0.heading = atof(tok); break;
		case 24:	data_updata0.UTC = atoi(tok); break;

		default:printf("error\n");
		}

		
	}//字符串拆分操作 

	
	free(tmp);

	fclose(stream);//文件打开后要进行关闭操作
}




void print_two_dimension(double** data, int row, int col)
{
	int i, j;
	for (i = 0; i<row; i++){
		for (j = 0; j<col; j++){
			printf("%f\t", data[i][j]);
		}
		printf("\n");
	}//打印的时候不打印第一行，第一行全是字符
}

/*
 * atoi,atol,strtod
*/









