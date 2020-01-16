
/****************************************************************************
*
* �ļ�����fuzzyPID.c
* ������ģ��PID������
* ���ߣ���������ģȺ Ⱥ�ѣ����� USTC
* ���ڣ�2017.7
* �޸ļ�¼��2018.1.6 jiezhi320
            1����ֹ������Ϊ0
			2��˫���ȸ����Ϊ�����ȸ���
*/


#include <math.h>
#include "stdint.h"
#include <float.h>
#include "fuzzyPID.h"

#define NB 0  //����
#define NM 1  //����
#define NS 2  //��С
#define ZO 3  //��
#define PS 4  //��С
#define PM 5  //����
#define PB 6  //����

static uint8_t is_zero(const float v)
{
    return fabsf(v) < FLT_EPSILON ? 1 : 0;
}

/****************************************************************************
*
* ������������������
*
****************************************************************************/

float uf(float x, float a, float  b, float c)
{
    if(x<=a)
        return 0.0f;
    else if((a<x)&&(x<=b))
    {
        if (is_zero(b-a))
            return 0.0f;
        else
            return  (x-a)/(b-a);
    }
    else if((b<x)&&(x<=c))
        return (c-x)/(c-b);
    else if(x>c)
        return 0.0f;
}

/****************************************************************************
*
* ������������������
*
****************************************************************************/

float ufl(float x,float a,float b)
{
    if(x<=a)
        return 1.0f;
    else if((a<x)&&(x<=b))
    {
        if (is_zero(b-a))
            return 0.0f;
        else
            return (b-x)/(b-a);
    }
    else if(x>b)
        return 0.0f;
}

/****************************************************************************
*
* ������������������
*
****************************************************************************/

float ufr(float x,float a,float b)
{
    if(x<=a)
        return 0.0f;
    if((a<x)&&(x<b))
    {
        if (is_zero(b-a))
            return 0.0f;
        else
            return (x-a)/(b-a);
    }
    if(x>=b)
        return 1.0f;
}


/****************************************************************************
*
* ȡС��������
*
****************************************************************************/

float fand(float a,float b)
{
    return (a<b)?a:b;
}

/****************************************************************************
*
* ȡ��������
*
****************************************************************************/

float forr(float a,float b)
{
    return (a<b)?b:a;
}

/****************************************************************************
*
*�ļ�����FUZZY_Get_pos_a(float e,float ec,float ethr,float ecthr)
*
*��  �ܣ�ģ���������������λ�õĺ�����
*
*��ڲ�����float e,float ec,float ethr,float ecthr
*          ���,���仯�������ֵ�����仯��ֵ
*
*���ڲ�����a
*
****************************************************************************/
int FUZZY_Get_pos_a(float e,float ec,float ethr,float ecthr)
{
    float es[7],ecs[7];  //���e�����仯��ec��Ӧģ���Ӽ���������
    float form[7][7]; //�����洢ijλ��e��ec��Ӧ�����������С��һ��
    int i,j;
    int a=0,b=0;

    es[NB]=ufl(e,-3*ethr,-1*ethr);        //�������e NB ��Ӧ��������
    es[NM]=uf(e,-3*ethr,-2*ethr,0);       //�������e NM ��Ӧ��������
    es[NS]=uf(e,-3*ethr,-1*ethr,1*ethr);  //�������e NS ��Ӧ��������
    es[ZO]=uf(e,-2*ethr,0,2*ethr);        //�������e ZO ��Ӧ��������
    es[PS]=uf(e,-1*ethr,1*ethr,3*ethr);   //�������e PS ��Ӧ��������
    es[PM]=uf(e,0,2*ethr,3*ethr);         //�������e PM ��Ӧ��������
    es[PB]=ufr(e,1*ethr,3*ethr);          //�������e PB ��Ӧ��������

    ecs[NB]=ufl(ec,-3*ecthr,-1*ecthr);        //�������ec NB ��Ӧ��������
    ecs[NM]=uf(ec,-3*ecthr,-2*ecthr,0);       //�������ec NM ��Ӧ��������
    ecs[NS]=uf(ec,-3*ecthr,-1*ecthr,1*ecthr); //�������ec NS ��Ӧ��������
    ecs[ZO]=uf(ec,-2*ecthr,0,2*ecthr);        //�������ec ZO ��Ӧ��������
    ecs[PS]=uf(ec,-1*ecthr,1*ecthr,3*ecthr);  //�������ec PS ��Ӧ��������
    ecs[PM]=uf(ec,0,2*ecthr,3*ecthr);         //�������ec PM ��Ӧ��������
    ecs[PB]=ufr(ec,1*ecthr,3*ecthr);          //�������ec PB ��Ӧ��������

    for(i=0; i<7; i++)
    {
        float w,h,r;
        for(j=0; j<7; j++)
        {
            h=es[i];
            r=ecs[j];
            w=fand(h,r);  //ȡС���洢ijλ��e��ec��Ӧ�����������С��һ��
            form[i][j]=w;
        }
    }

    for(i=0; i<7; i++)
    {
        for(j=0; j<7; j++)
        {
            if(form[a][b]<form[i][j])   //ȡ��Ѱ�ҹ������������������һ��
            {
                a=i;
                b=j;
            }
        }
    }
    return a;
}

/****************************************************************************
*
*�ļ�����FUZZY_Get_pos_b(float e,float ec,float ethr,float ecthr)
*
*��  �ܣ�ģ���������������λ�õ�������
*
*��ڲ�����float e,float ec,float ethr,float ecthr
*          ���,���仯�������ֵ�����仯��ֵ
*
*���ڲ�����b
*
****************************************************************************/
int FUZZY_Get_pos_b(float e,float ec,float ethr,float ecthr)
{
    float es[7],ecs[7];  //���e�����仯��ec��Ӧģ���Ӽ���������
    float form[7][7]; //�����洢ijλ��e��ec��Ӧ�����������С��һ��
    int i,j;
    int a=0,b=0;

    es[NB]=ufl(e,-3*ethr,-1*ethr);        //�������e NB ��Ӧ��������
    es[NM]=uf(e,-3*ethr,-2*ethr,0);       //�������e NM ��Ӧ��������
    es[NS]=uf(e,-3*ethr,-1*ethr,1*ethr);  //�������e NS ��Ӧ��������
    es[ZO]=uf(e,-2*ethr,0,2*ethr);        //�������e ZO ��Ӧ��������
    es[PS]=uf(e,-1*ethr,1*ethr,3*ethr);   //�������e PS ��Ӧ��������
    es[PM]=uf(e,0,2*ethr,3*ethr);         //�������e PM ��Ӧ��������
    es[PB]=ufr(e,1*ethr,3*ethr);          //�������e PB ��Ӧ��������

    ecs[NB]=ufl(ec,-3*ecthr,-1*ecthr);        //�������ec NB ��Ӧ��������
    ecs[NM]=uf(ec,-3*ecthr,-2*ecthr,0);       //�������ec NM ��Ӧ��������
    ecs[NS]=uf(ec,-3*ecthr,-1*ecthr,1*ecthr); //�������ec NS ��Ӧ��������
    ecs[ZO]=uf(ec,-2*ecthr,0,2*ecthr);        //�������ec ZO ��Ӧ��������
    ecs[PS]=uf(ec,-1*ecthr,1*ecthr,3*ecthr);  //�������ec PS ��Ӧ��������
    ecs[PM]=uf(ec,0,2*ecthr,3*ecthr);         //�������ec PM ��Ӧ��������
    ecs[PB]=ufr(ec,1*ecthr,3*ecthr);          //�������ec PB ��Ӧ��������
    for(i=0; i<7; i++)
    {
        float w,h,r;
        for(j=0; j<7; j++)
        {
            h=es[i];
            r=ecs[j];
            w=fand(h,r);  //ȡС���洢ijλ��e��ec��Ӧ�����������С��һ��
            form[i][j]=w;
        }
    }

    for(i=0; i<7; i++)
    {
        for(j=0; j<7; j++)
        {
            if(form[a][b]<form[i][j])   //ȡ��Ѱ�ҹ������������������һ��
            {
                a=i;
                b=j;
            }
        }
    }
    return b;
}

/****************************************************************************
*
*�ļ�����FUZZY_Get_lsd(float e,float ec,float ethr,float ecthr)
*
*��  �ܣ�ģ��������{e,ec}������С�����ȵ����ֵ
*
*��ڲ�����float e,float ec,float ethr,float ecthr
*          ���,���仯�������ֵ�����仯��ֵ
*
*���ڲ�����lsd
*
****************************************************************************/

float FUZZY_Get_pos_lsd(float e,float ec,float ethr,float ecthr)
{
    float es[7],ecs[7];  //���e�����仯��ec��Ӧģ���Ӽ���������
    float form[7][7]; //�����洢ijλ��e��ec��Ӧ�����������С��һ��
    int i,j;
    int a=0,b=0;
    float lsd=0;

    es[NB]=ufl(e,-3*ethr,-1*ethr);        //�������e NB ��Ӧ��������
    es[NM]=uf(e,-3*ethr,-2*ethr,0);       //�������e NM ��Ӧ��������
    es[NS]=uf(e,-3*ethr,-1*ethr,1*ethr);  //�������e NS ��Ӧ��������
    es[ZO]=uf(e,-2*ethr,0,2*ethr);        //�������e ZO ��Ӧ��������
    es[PS]=uf(e,-1*ethr,1*ethr,3*ethr);   //�������e PS ��Ӧ��������
    es[PM]=uf(e,0,2*ethr,3*ethr);         //�������e PM ��Ӧ��������
    es[PB]=ufr(e,1*ethr,3*ethr);          //�������e PB ��Ӧ��������

    ecs[NB]=ufl(ec,-3*ecthr,-1*ecthr);        //�������ec NB ��Ӧ��������
    ecs[NM]=uf(ec,-3*ecthr,-2*ecthr,0);       //�������ec NM ��Ӧ��������
    ecs[NS]=uf(ec,-3*ecthr,-1*ecthr,1*ecthr); //�������ec NS ��Ӧ��������
    ecs[ZO]=uf(ec,-2*ecthr,0,2*ecthr);        //�������ec ZO ��Ӧ��������
    ecs[PS]=uf(ec,-1*ecthr,1*ecthr,3*ecthr);  //�������ec PS ��Ӧ��������
    ecs[PM]=uf(ec,0,2*ecthr,3*ecthr);         //�������ec PM ��Ӧ��������
    ecs[PB]=ufr(ec,1*ecthr,3*ecthr);          //�������ec PB ��Ӧ��������
    for(i=0; i<7; i++)

        for(i=0; i<7; i++)
        {
            float w,h,r;
            for(j=0; j<7; j++)
            {
                h=es[i];
                r=ecs[j];
                w=fand(h,r);  //ȡС���洢ijλ��e��ec��Ӧ�����������С��һ��
                form[i][j]=w;
            }
        }

    for(i=0; i<7; i++)
    {
        for(j=0; j<7; j++)
        {
            if(form[a][b]<form[i][j])   //ȡ��Ѱ�ҹ������������������һ��
            {
                a=i;
                b=j;
            }
        }
    }
    lsd=form[a][b];
    return lsd;
}

const int kprule[7][7]= {{PB,PB,PM,PM,PS,ZO,ZO}, //kp��ģ�������
	{PB,PB,PM,PS,PS,ZO,ZO},
	{PM,PM,PM,PS,ZO,NS,NS},
	{PM,PM,PS,ZO,NS,NM,NM},
	{PS,PS,ZO,NS,NS,NM,NM},
	{PS,ZO,NS,NM,NM,NM,NB},
	{ZO,ZO,NM,NM,NM,NB,NB}
};
/****************************************************************************
*
*�ļ�����FUZZY_Calc_detKp(float e,float ec,float ethr,float ecthr,float kpthr)
*
*��  �ܣ�ģ����detKp���㲿��
*
*��ڲ�����float e,float ec,float ethr,float ecthr,float kpthr
*          ���,���仯�������ֵ�����仯��ֵ��Kp�仯������ֵ
*
*���ڲ�����detKp
*
****************************************************************************/
float FUZZY_Calc_detKp(float e,float ec,float ethr,float ecthr,float kpthr)
{
    int a=0,b=0,kpr=0;
    float detkp=0,lsd=0;

    a=FUZZY_Get_pos_a(e,ec,ethr,ecthr);      //ѡȡkp�����ĳһ����λ�õĺ�����
    b=FUZZY_Get_pos_b(e,ec,ethr,ecthr);      //ѡȡkp�����ĳһ����λ�õ�������
    lsd=FUZZY_Get_pos_lsd(e,ec,ethr,ecthr);  //ģ��������{e,ec}������С�����ȵ����ֵ
    kpr=kprule[a][b];                        //ѡȡkp����


    if(kpr==NB)
        detkp=ufl(lsd,-3*kpthr,-1*kpthr);
    else if(kpr==NM)
        detkp=uf(lsd,-3*kpthr,2*kpthr,0);
    else if(kpr==NS)
        detkp=uf(lsd,-3*kpthr,1*kpthr,1*kpthr);
    else if(kpr==ZO)
        detkp=uf(lsd,-2*kpthr,0,2*kpthr);
    else if(kpr==PS)
        detkp=uf(lsd,-1*kpthr,1*kpthr,3*kpthr);
    else if(kpr==PM)
        detkp=uf(lsd,0,2*kpthr,3*kpthr);
    else if(kpr==PB)
        detkp=ufr(lsd,1*kpthr,3*kpthr);

    return detkp;
}


const int kirule[7][7]= {{NB,NB,NM,NM,NS,ZO,ZO},    //ki��ģ�������
        {NB,NB,NM,NS,NS,ZO,ZO},
        {NB,NM,NS,NS,ZO,PS,PS},
        {NM,NM,NS,ZO,PS,PM,PM},
        {NM,NS,ZO,PS,PS,PM,PB},
        {ZO,ZO,PS,PS,PM,PB,PB},
        {ZO,ZO,PS,PM,PM,PB,PB}
    };
/****************************************************************************
*
*�ļ�����FUZZY_Calc_detKi(float e,float ec,float ethr,float ecthr,float detkithr)
*
*��  �ܣ�ģ����detKi���㲿��
*
*��ڲ�����float e,float ec,float ethr,float ecthr,float detkithr
*          ���,���仯�������ֵ�����仯��ֵ��Ki�仯������ֵ
*
*���ڲ�����detKi
*
****************************************************************************/
float FUZZY_Calc_detKi(float e,float ec,float ethr,float ecthr,float detkithr)
{

    int a=0,b=0,kir=0;
    float detki=0,lsd=0;

    a=FUZZY_Get_pos_a(e,ec,ethr,ecthr);      //ѡȡki�����ĳһ����λ�õĺ�����
    b=FUZZY_Get_pos_b(e,ec,ethr,ecthr);      //ѡȡki�����ĳһ����λ�õ�������
    lsd=FUZZY_Get_pos_lsd(e,ec,ethr,ecthr);  //ģ��������{e,ec}������С�����ȵ����ֵ
    kir=kirule[a][b];                        //ѡȡki����


    if(kir==NB)
        detki=ufl(lsd,-3*detkithr,-2*detkithr);
    else if(kir==NM)
        detki=uf(lsd,-3*detkithr,-2*detkithr,0);
    else if(kir==NS)
        detki=uf(lsd,-3*detkithr,-1*detkithr,1*detkithr);
    else if(kir==ZO)
        detki=uf(lsd,-2*detkithr,0,2*detkithr);
    else if(kir==PS)
        detki=uf(lsd,-1*detkithr,1*detkithr,3*detkithr);
    else if(kir==PM)
        detki=uf(lsd,0,2*detkithr,3*detkithr);
    else if (kir==PB)
        detki=ufr(lsd,1*detkithr,3*detkithr);

    return detki;
}

const  int kdrule[7][7]= {{PS,NS,NB,NB,NB,NM,PS},   //kd��ģ�������
        {PS,NS,NB,NM,NM,NS,ZO},
        {ZO,NS,NM,NM,NS,NS,ZO},
        {ZO,NS,NS,NS,NS,NS,ZO},
        {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
        {PB,NS,PS,PS,PS,PS,PB},
        {PB,PM,PM,PM,PS,PS,PB}
    };
/****************************************************************************
*
*�ļ�����FUZZY_Calc_detKd(float e,float ec,float ethr,float ecthr,float detdetkdthr)
*
*��  �ܣ�ģ����detKd���㲿��
*
*��ڲ�����float e,float ec,float ethr,float ecthr,float detdetkdthr
*          ���,���仯�������ֵ�����仯��ֵ��Kd�仯������ֵ
*
*���ڲ�����detKd
*
****************************************************************************/
float FUZZY_Calc_detKd(float e,float ec,float ethr,float ecthr,float detdetkdthr)
{

    int a=0,b=0,kdr=0;
    float detkd=0,lsd=0;

    a=FUZZY_Get_pos_a(e,ec,ethr,ecthr);      //ѡȡkd�����ĳһ����λ�õĺ�����
    b=FUZZY_Get_pos_b(e,ec,ethr,ecthr);      //ѡȡkd�����ĳһ����λ�õ�������
    lsd=FUZZY_Get_pos_lsd(e,ec,ethr,ecthr);  //ģ��������{e,ec}������С�����ȵ����ֵ
    kdr=kdrule[a][b];                        //ѡȡkd����


    if(kdr==NB)
        detkd=ufl(lsd,-3*detdetkdthr,-1*detdetkdthr);
    else if(kdr==NM)
        detkd=uf(lsd,-3*detdetkdthr,2*detdetkdthr,0);
    else if(kdr==NS)
        detkd=uf(lsd,-3*detdetkdthr,1*detdetkdthr,1*detdetkdthr);
    else if(kdr==ZO)
        detkd=uf(lsd,-2*detdetkdthr,0,2*detdetkdthr);
    else if(kdr==PS)
        detkd=uf(lsd,-1*detdetkdthr,1*detdetkdthr,3*detdetkdthr);
    else if(kdr==PM)
        detkd=uf(lsd,0,2*detdetkdthr,3*detdetkdthr);
    else if(kdr==PB)
        detkd=ufr(lsd,1*detdetkdthr,3*detdetkdthr);

    return detkd;
}

//void main()
//{
//    float kp=1,ki=0.05,kd=1;  //PID�����ĳ�ʼֵ
//    float detkp=0,detki=0,detkd=0;  //ͨ��ģ������ó�
//	float ethr=1,ecthr=0.1,kpthr=0.1,kithr=0.02,kdthr=1; //���,���仯��Kp,Ki,Kd�仯������ֵ
//    float e,ec=0,sume=0,e1=0;
//    float u=0,start,end;
//	int ii=0;

//	start=clock();

//    while(ii<50000)
//    {
//		//printf("input a error value:");
//        //scanf("%f\n",&_e);
//		//printf("\ntest\n");
//		ii++;
//		e=0.1;
//        sume+=e;
//        ec=e-e1;
//        e1=e;

//        detkp=FUZZY_Calc_detKp(e,ec,ethr,ecthr,kpthr);  //����detKp
//        detki=FUZZY_Calc_detKi(e,ec,ethr,ecthr,kithr);  //����detKi
//        detkd=FUZZY_Calc_detKd(e,ec,ethr,ecthr,kdthr);  //����detKd

//        kp+=detkp;   //���㵱ǰkp
//        ki+=detki;   //���㵱ǰki
//        kd+=detkd;   //���㵱ǰkd
//		//printf("control output : %f\n\n",u);
//        u=kp*e+ki*sume+kd*ec;   //PID���������

//        if(u>=10)   //�޷�
//        {
//            u=10;
//        }
//        if(u<=0)
//        {
//            u=0;
//        }
//	//printf(" e:%f\n ec:%f\n kp:%f\n ki:%f\n kd:%f\n control output : %f\n\n",e,ec,kp,ki,kd,u);
//    }

//	end=clock();
//	printf("\n\nrunning time:%f\n\n",(end-start)/CLOCKS_PER_SEC);
//}