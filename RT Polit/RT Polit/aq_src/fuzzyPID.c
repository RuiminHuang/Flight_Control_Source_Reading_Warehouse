
/****************************************************************************
*
* 文件名：fuzzyPID.c
* 描述：模糊PID控制器
* 作者：反重力航模群 群友：杨柳 USTC
* 日期：2017.7
* 修改记录：2018.1.6 jiezhi320
            1、防止被除数为0
			2、双精度浮点改为单精度浮点
*/


#include <math.h>
#include "stdint.h"
#include <float.h>
#include "fuzzyPID.h"

#define NB 0  //负大
#define NM 1  //负中
#define NS 2  //负小
#define ZO 3  //零
#define PS 4  //正小
#define PM 5  //正中
#define PB 6  //正大

static uint8_t is_zero(const float v)
{
    return fabsf(v) < FLT_EPSILON ? 1 : 0;
}

/****************************************************************************
*
* 三角形隶属函数描述
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
* 左梯形隶属函数描述
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
* 右梯形隶属函数描述
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
* 取小函数描述
*
****************************************************************************/

float fand(float a,float b)
{
    return (a<b)?a:b;
}

/****************************************************************************
*
* 取大函数描述
*
****************************************************************************/

float forr(float a,float b)
{
    return (a<b)?b:a;
}

/****************************************************************************
*
*文件名：FUZZY_Get_pos_a(float e,float ec,float ethr,float ecthr)
*
*功  能：模糊规则隶属度最大位置的横坐标
*
*入口参数：float e,float ec,float ethr,float ecthr
*          误差,误差变化，误差阈值，误差变化阈值
*
*出口参数：a
*
****************************************************************************/
int FUZZY_Get_pos_a(float e,float ec,float ethr,float ecthr)
{
    float es[7],ecs[7];  //误差e和误差变化率ec对应模糊子集的隶属度
    float form[7][7]; //用来存储ij位置e和ec对应隶属度中相对小的一个
    int i,j;
    int a=0,b=0;

    es[NB]=ufl(e,-3*ethr,-1*ethr);        //定义误差e NB 对应的隶属度
    es[NM]=uf(e,-3*ethr,-2*ethr,0);       //定义误差e NM 对应的隶属度
    es[NS]=uf(e,-3*ethr,-1*ethr,1*ethr);  //定义误差e NS 对应的隶属度
    es[ZO]=uf(e,-2*ethr,0,2*ethr);        //定义误差e ZO 对应的隶属度
    es[PS]=uf(e,-1*ethr,1*ethr,3*ethr);   //定义误差e PS 对应的隶属度
    es[PM]=uf(e,0,2*ethr,3*ethr);         //定义误差e PM 对应的隶属度
    es[PB]=ufr(e,1*ethr,3*ethr);          //定义误差e PB 对应的隶属度

    ecs[NB]=ufl(ec,-3*ecthr,-1*ecthr);        //定义误差ec NB 对应的隶属度
    ecs[NM]=uf(ec,-3*ecthr,-2*ecthr,0);       //定义误差ec NM 对应的隶属度
    ecs[NS]=uf(ec,-3*ecthr,-1*ecthr,1*ecthr); //定义误差ec NS 对应的隶属度
    ecs[ZO]=uf(ec,-2*ecthr,0,2*ecthr);        //定义误差ec ZO 对应的隶属度
    ecs[PS]=uf(ec,-1*ecthr,1*ecthr,3*ecthr);  //定义误差ec PS 对应的隶属度
    ecs[PM]=uf(ec,0,2*ecthr,3*ecthr);         //定义误差ec PM 对应的隶属度
    ecs[PB]=ufr(ec,1*ecthr,3*ecthr);          //定义误差ec PB 对应的隶属度

    for(i=0; i<7; i++)
    {
        float w,h,r;
        for(j=0; j<7; j++)
        {
            h=es[i];
            r=ecs[j];
            w=fand(h,r);  //取小，存储ij位置e和ec对应隶属度中相对小的一个
            form[i][j]=w;
        }
    }

    for(i=0; i<7; i++)
    {
        for(j=0; j<7; j++)
        {
            if(form[a][b]<form[i][j])   //取大，寻找规则表中隶属度中最大的一个
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
*文件名：FUZZY_Get_pos_b(float e,float ec,float ethr,float ecthr)
*
*功  能：模糊规则隶属度最大位置的纵坐标
*
*入口参数：float e,float ec,float ethr,float ecthr
*          误差,误差变化，误差阈值，误差变化阈值
*
*出口参数：b
*
****************************************************************************/
int FUZZY_Get_pos_b(float e,float ec,float ethr,float ecthr)
{
    float es[7],ecs[7];  //误差e和误差变化率ec对应模糊子集的隶属度
    float form[7][7]; //用来存储ij位置e和ec对应隶属度中相对小的一个
    int i,j;
    int a=0,b=0;

    es[NB]=ufl(e,-3*ethr,-1*ethr);        //定义误差e NB 对应的隶属度
    es[NM]=uf(e,-3*ethr,-2*ethr,0);       //定义误差e NM 对应的隶属度
    es[NS]=uf(e,-3*ethr,-1*ethr,1*ethr);  //定义误差e NS 对应的隶属度
    es[ZO]=uf(e,-2*ethr,0,2*ethr);        //定义误差e ZO 对应的隶属度
    es[PS]=uf(e,-1*ethr,1*ethr,3*ethr);   //定义误差e PS 对应的隶属度
    es[PM]=uf(e,0,2*ethr,3*ethr);         //定义误差e PM 对应的隶属度
    es[PB]=ufr(e,1*ethr,3*ethr);          //定义误差e PB 对应的隶属度

    ecs[NB]=ufl(ec,-3*ecthr,-1*ecthr);        //定义误差ec NB 对应的隶属度
    ecs[NM]=uf(ec,-3*ecthr,-2*ecthr,0);       //定义误差ec NM 对应的隶属度
    ecs[NS]=uf(ec,-3*ecthr,-1*ecthr,1*ecthr); //定义误差ec NS 对应的隶属度
    ecs[ZO]=uf(ec,-2*ecthr,0,2*ecthr);        //定义误差ec ZO 对应的隶属度
    ecs[PS]=uf(ec,-1*ecthr,1*ecthr,3*ecthr);  //定义误差ec PS 对应的隶属度
    ecs[PM]=uf(ec,0,2*ecthr,3*ecthr);         //定义误差ec PM 对应的隶属度
    ecs[PB]=ufr(ec,1*ecthr,3*ecthr);          //定义误差ec PB 对应的隶属度
    for(i=0; i<7; i++)
    {
        float w,h,r;
        for(j=0; j<7; j++)
        {
            h=es[i];
            r=ecs[j];
            w=fand(h,r);  //取小，存储ij位置e和ec对应隶属度中相对小的一个
            form[i][j]=w;
        }
    }

    for(i=0; i<7; i++)
    {
        for(j=0; j<7; j++)
        {
            if(form[a][b]<form[i][j])   //取大，寻找规则表中隶属度中最大的一个
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
*文件名：FUZZY_Get_lsd(float e,float ec,float ethr,float ecthr)
*
*功  能：模糊规则中{e,ec}两者最小隶属度的最大值
*
*入口参数：float e,float ec,float ethr,float ecthr
*          误差,误差变化，误差阈值，误差变化阈值
*
*出口参数：lsd
*
****************************************************************************/

float FUZZY_Get_pos_lsd(float e,float ec,float ethr,float ecthr)
{
    float es[7],ecs[7];  //误差e和误差变化率ec对应模糊子集的隶属度
    float form[7][7]; //用来存储ij位置e和ec对应隶属度中相对小的一个
    int i,j;
    int a=0,b=0;
    float lsd=0;

    es[NB]=ufl(e,-3*ethr,-1*ethr);        //定义误差e NB 对应的隶属度
    es[NM]=uf(e,-3*ethr,-2*ethr,0);       //定义误差e NM 对应的隶属度
    es[NS]=uf(e,-3*ethr,-1*ethr,1*ethr);  //定义误差e NS 对应的隶属度
    es[ZO]=uf(e,-2*ethr,0,2*ethr);        //定义误差e ZO 对应的隶属度
    es[PS]=uf(e,-1*ethr,1*ethr,3*ethr);   //定义误差e PS 对应的隶属度
    es[PM]=uf(e,0,2*ethr,3*ethr);         //定义误差e PM 对应的隶属度
    es[PB]=ufr(e,1*ethr,3*ethr);          //定义误差e PB 对应的隶属度

    ecs[NB]=ufl(ec,-3*ecthr,-1*ecthr);        //定义误差ec NB 对应的隶属度
    ecs[NM]=uf(ec,-3*ecthr,-2*ecthr,0);       //定义误差ec NM 对应的隶属度
    ecs[NS]=uf(ec,-3*ecthr,-1*ecthr,1*ecthr); //定义误差ec NS 对应的隶属度
    ecs[ZO]=uf(ec,-2*ecthr,0,2*ecthr);        //定义误差ec ZO 对应的隶属度
    ecs[PS]=uf(ec,-1*ecthr,1*ecthr,3*ecthr);  //定义误差ec PS 对应的隶属度
    ecs[PM]=uf(ec,0,2*ecthr,3*ecthr);         //定义误差ec PM 对应的隶属度
    ecs[PB]=ufr(ec,1*ecthr,3*ecthr);          //定义误差ec PB 对应的隶属度
    for(i=0; i<7; i++)

        for(i=0; i<7; i++)
        {
            float w,h,r;
            for(j=0; j<7; j++)
            {
                h=es[i];
                r=ecs[j];
                w=fand(h,r);  //取小，存储ij位置e和ec对应隶属度中相对小的一个
                form[i][j]=w;
            }
        }

    for(i=0; i<7; i++)
    {
        for(j=0; j<7; j++)
        {
            if(form[a][b]<form[i][j])   //取大，寻找规则表中隶属度中最大的一个
            {
                a=i;
                b=j;
            }
        }
    }
    lsd=form[a][b];
    return lsd;
}

const int kprule[7][7]= {{PB,PB,PM,PM,PS,ZO,ZO}, //kp的模糊规则表
	{PB,PB,PM,PS,PS,ZO,ZO},
	{PM,PM,PM,PS,ZO,NS,NS},
	{PM,PM,PS,ZO,NS,NM,NM},
	{PS,PS,ZO,NS,NS,NM,NM},
	{PS,ZO,NS,NM,NM,NM,NB},
	{ZO,ZO,NM,NM,NM,NB,NB}
};
/****************************************************************************
*
*文件名：FUZZY_Calc_detKp(float e,float ec,float ethr,float ecthr,float kpthr)
*
*功  能：模糊算detKp计算部分
*
*入口参数：float e,float ec,float ethr,float ecthr,float kpthr
*          误差,误差变化，误差阈值，误差变化阈值，Kp变化量的阈值
*
*出口参数：detKp
*
****************************************************************************/
float FUZZY_Calc_detKp(float e,float ec,float ethr,float ecthr,float kpthr)
{
    int a=0,b=0,kpr=0;
    float detkp=0,lsd=0;

    a=FUZZY_Get_pos_a(e,ec,ethr,ecthr);      //选取kp规则表某一规则位置的横坐标
    b=FUZZY_Get_pos_b(e,ec,ethr,ecthr);      //选取kp规则表某一规则位置的纵坐标
    lsd=FUZZY_Get_pos_lsd(e,ec,ethr,ecthr);  //模糊规则中{e,ec}两者最小隶属度的最大值
    kpr=kprule[a][b];                        //选取kp规则


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


const int kirule[7][7]= {{NB,NB,NM,NM,NS,ZO,ZO},    //ki的模糊规则表
        {NB,NB,NM,NS,NS,ZO,ZO},
        {NB,NM,NS,NS,ZO,PS,PS},
        {NM,NM,NS,ZO,PS,PM,PM},
        {NM,NS,ZO,PS,PS,PM,PB},
        {ZO,ZO,PS,PS,PM,PB,PB},
        {ZO,ZO,PS,PM,PM,PB,PB}
    };
/****************************************************************************
*
*文件名：FUZZY_Calc_detKi(float e,float ec,float ethr,float ecthr,float detkithr)
*
*功  能：模糊算detKi计算部分
*
*入口参数：float e,float ec,float ethr,float ecthr,float detkithr
*          误差,误差变化，误差阈值，误差变化阈值，Ki变化量的阈值
*
*出口参数：detKi
*
****************************************************************************/
float FUZZY_Calc_detKi(float e,float ec,float ethr,float ecthr,float detkithr)
{

    int a=0,b=0,kir=0;
    float detki=0,lsd=0;

    a=FUZZY_Get_pos_a(e,ec,ethr,ecthr);      //选取ki规则表某一规则位置的横坐标
    b=FUZZY_Get_pos_b(e,ec,ethr,ecthr);      //选取ki规则表某一规则位置的纵坐标
    lsd=FUZZY_Get_pos_lsd(e,ec,ethr,ecthr);  //模糊规则中{e,ec}两者最小隶属度的最大值
    kir=kirule[a][b];                        //选取ki规则


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

const  int kdrule[7][7]= {{PS,NS,NB,NB,NB,NM,PS},   //kd的模糊规则表
        {PS,NS,NB,NM,NM,NS,ZO},
        {ZO,NS,NM,NM,NS,NS,ZO},
        {ZO,NS,NS,NS,NS,NS,ZO},
        {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
        {PB,NS,PS,PS,PS,PS,PB},
        {PB,PM,PM,PM,PS,PS,PB}
    };
/****************************************************************************
*
*文件名：FUZZY_Calc_detKd(float e,float ec,float ethr,float ecthr,float detdetkdthr)
*
*功  能：模糊算detKd计算部分
*
*入口参数：float e,float ec,float ethr,float ecthr,float detdetkdthr
*          误差,误差变化，误差阈值，误差变化阈值，Kd变化量的阈值
*
*出口参数：detKd
*
****************************************************************************/
float FUZZY_Calc_detKd(float e,float ec,float ethr,float ecthr,float detdetkdthr)
{

    int a=0,b=0,kdr=0;
    float detkd=0,lsd=0;

    a=FUZZY_Get_pos_a(e,ec,ethr,ecthr);      //选取kd规则表某一规则位置的横坐标
    b=FUZZY_Get_pos_b(e,ec,ethr,ecthr);      //选取kd规则表某一规则位置的纵坐标
    lsd=FUZZY_Get_pos_lsd(e,ec,ethr,ecthr);  //模糊规则中{e,ec}两者最小隶属度的最大值
    kdr=kdrule[a][b];                        //选取kd规则


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
//    float kp=1,ki=0.05,kd=1;  //PID参数的初始值
//    float detkp=0,detki=0,detkd=0;  //通过模糊推理得出
//	float ethr=1,ecthr=0.1,kpthr=0.1,kithr=0.02,kdthr=1; //误差,误差变化，Kp,Ki,Kd变化量的阈值
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

//        detkp=FUZZY_Calc_detKp(e,ec,ethr,ecthr,kpthr);  //计算detKp
//        detki=FUZZY_Calc_detKi(e,ec,ethr,ecthr,kithr);  //计算detKi
//        detkd=FUZZY_Calc_detKd(e,ec,ethr,ecthr,kdthr);  //计算detKd

//        kp+=detkp;   //计算当前kp
//        ki+=detki;   //计算当前ki
//        kd+=detkd;   //计算当前kd
//		//printf("control output : %f\n\n",u);
//        u=kp*e+ki*sume+kd*ec;   //PID控制器输出

//        if(u>=10)   //限幅
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