
#if 0
int main(void)
{
	FILE *fp = NULL;
	char *line, *record;
	char buffer[1024];
	if ((fp = fopen("Student.csv", "at+")) != NULL)
	{
		fseek(fp, 170L, SEEK_SET);  //定位到第二行，每个英文字符大小为1
		char delims[] = ",";
		char *result = NULL;
		int j = 0;
		while ((line = fgets(buffer, sizeof(buffer), fp)) != NULL)//当没有读取到文件末尾时循环继续
		{
			record = strtok(line, ",");
			while (record != NULL)//读取每一行的数据
			{
				if (strcmp(record, "Ps:") == 0)//当读取到Ps那一行时，不再继续读取
					return 0;
				printf("%s ", record);//将读取到的每一个数据打印出来
				if (j == 10)  //只需读取前9列
					break;
				record = strtok(NULL, ",");
				j++;
			}
			printf("\n");
			j = 0;

		}
		fclose(fp);
		fp = NULL;
	}



	return 0;
}








#endif




#if 0
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>  
#include "kalman_filter.h" 
//常数
float phi = 45.7796; // 经度（哈尔滨）
float lamda = 126.6705;//纬度（哈尔滨）
float deg2rad = 3.1415926 / 180; //角度变弧度
float rad2deg = 180 / 3.1415926; //弧度变角度



float g = 9.87032;
float ts = 0.01; //时间间隔（由采样频率决定）
float Freq = 100;
float dt = 0.01;

// attitude 和四元数
float q_system[4] = { 0 };
float att_system[3] = { 0 };

int main()
{
	float data[15] = { 0 };
	int i = 0;
	FILE *fpRead = fopen("C:\\Users\\Administrator\\Desktop\\2.csv", "r");

	phi = phi * deg2rad;
	lamda = lamda * deg2rad;
	if (fpRead == NULL)
	{
		printf("Error");
		return 0;

	}

	for (i = 0; i < 15; i++)
	{
		fscanf(fpRead, "%f", &data[i]);
		fseek(fpRead, 1L, SEEK_CUR);   /*fp指针从当前位置向后移动*/
	}


	/* For 6 Dimension 二维 */
	kalman6_state state6;
	float init_x[6];
	float init_p[6][6] = { { 1e-5, 0, 0, 0, 0, 0 }, { 0, 1e-5, 0, 0, 0, 0 }, { 0, 0, 1e-5, 0, 0, 0 }, { 0, 0, 0, 1e-7, 0, 0 }, { 0, 0, 0, 0, 1e-7, 0 }, { 0, 0, 0, 0, 0, 1e-7 } };
	float out2 = 0;

	init_x[0] = data[0];
	init_x[1] = data[1] - data[0];








	printf("Hello World");

	getchar();

	return 0;

}


#endif



