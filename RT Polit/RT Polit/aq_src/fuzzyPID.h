
#ifndef __FUZZY_PID_H_
#define __FUZZY_PID_H_



typedef struct {
    float ethr;//�����ֵ
	float ecthr;//���仯�˵���ֵ
		
	float kpthr;//KP�仯�˵���ֵ
	float kithr;//KI�仯�˵���ֵ
    float kdthr;//KD�仯�˵���ֵ	

    float kp,ki,kd; 	
		
}FUZZY_PID;


extern FUZZY_PID rate_fuzzy_pid[2];

float FUZZY_Calc_detKp(float e,float ec,float ethr,float ecthr,float kpthr);
float FUZZY_Calc_detKi(float e,float ec,float ethr,float ecthr,float detkithr);
float FUZZY_Calc_detKd(float e,float ec,float ethr,float ecthr,float detdetkdthr);




#endif
