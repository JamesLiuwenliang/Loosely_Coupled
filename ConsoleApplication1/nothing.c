
#if 0
int main(void)
{
	FILE *fp = NULL;
	char *line, *record;
	char buffer[1024];
	if ((fp = fopen("Student.csv", "at+")) != NULL)
	{
		fseek(fp, 170L, SEEK_SET);  //��λ���ڶ��У�ÿ��Ӣ���ַ���СΪ1
		char delims[] = ",";
		char *result = NULL;
		int j = 0;
		while ((line = fgets(buffer, sizeof(buffer), fp)) != NULL)//��û�ж�ȡ���ļ�ĩβʱѭ������
		{
			record = strtok(line, ",");
			while (record != NULL)//��ȡÿһ�е�����
			{
				if (strcmp(record, "Ps:") == 0)//����ȡ��Ps��һ��ʱ�����ټ�����ȡ
					return 0;
				printf("%s ", record);//����ȡ����ÿһ�����ݴ�ӡ����
				if (j == 10)  //ֻ���ȡǰ9��
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
//����
float phi = 45.7796; // ���ȣ���������
float lamda = 126.6705;//γ�ȣ���������
float deg2rad = 3.1415926 / 180; //�Ƕȱ仡��
float rad2deg = 180 / 3.1415926; //���ȱ�Ƕ�



float g = 9.87032;
float ts = 0.01; //ʱ�������ɲ���Ƶ�ʾ�����
float Freq = 100;
float dt = 0.01;

// attitude ����Ԫ��
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
		fseek(fpRead, 1L, SEEK_CUR);   /*fpָ��ӵ�ǰλ������ƶ�*/
	}


	/* For 6 Dimension ��ά */
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



