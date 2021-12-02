/*
  Developed by: Patrice Gama
  Date: 2020
  
  Active disturbance rejection control(ADRC) is a control algorithm which takes the characteristics of PID control with its improvement over PID comprising of three parts: 
Tracking differentiator (TD), extended state observer (ESO), and state error feedback

*/

#include "ADRC.h"

// Add in ADRC gains!
const float ADRC_Unit[3][16]=
{
/*TD
/*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0},
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0},
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   1.2,      0.0005,    5,    5,    0.8,   1.5,    50,    0},
};


int16_t Sign_ADRC(float Input)
{
    int16_t output=0;
    if(Input>1E-6) output=1;
    else if(Input<-1E-6) output=-1;
    else output=0;
    return output;
}

int16_t Fsg_ADRC(float x,float d)
{
  int16_t output=0;
  output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
  return output;
}

//****************************Init Param*********************

void ADRC_Init(Fhan_Data *fhan_in1,Fhan_Data *fhan_in2)
{
  fhan_in1->r=ADRC_Unit[0][0];
  fhan_in1->h=ADRC_Unit[0][1];
  fhan_in1->N0=(uint16)(ADRC_Unit[0][2]);
  fhan_in1->beta_01=ADRC_Unit[0][3];
  fhan_in1->beta_02=ADRC_Unit[0][4];
  fhan_in1->beta_03=ADRC_Unit[0][5];
  fhan_in1->b0=ADRC_Unit[0][6];
  fhan_in1->beta_0=ADRC_Unit[0][7];
  fhan_in1->beta_1=ADRC_Unit[0][8];
  fhan_in1->beta_2=ADRC_Unit[0][9];
  fhan_in1->N1=(uint16)(ADRC_Unit[0][10]);
  fhan_in1->c=ADRC_Unit[0][11];
  fhan_in1->alpha1=ADRC_Unit[0][12];
  fhan_in1->alpha2=ADRC_Unit[0][13];
  fhan_in1->zeta=ADRC_Unit[0][14];
  fhan_in1->b=ADRC_Unit[0][15];
  
  fhan_in2->r=ADRC_Unit[1][0];
  fhan_in2->h=ADRC_Unit[1][1];
  fhan_in2->N0=(uint16)(ADRC_Unit[1][2]);
  fhan_in2->beta_01=ADRC_Unit[1][3];
  fhan_in2->beta_02=ADRC_Unit[1][4];
  fhan_in2->beta_03=ADRC_Unit[1][5];
  fhan_in2->b0=ADRC_Unit[1][6];
  fhan_in2->beta_0=ADRC_Unit[1][7];
  fhan_in2->beta_1=ADRC_Unit[1][8];
  fhan_in2->beta_2=ADRC_Unit[1][9];
  fhan_in2->N1=(uint16)(ADRC_Unit[1][10]);
  fhan_in2->c=ADRC_Unit[1][11];
  fhan_in2->alpha1=ADRC_Unit[1][12];
  fhan_in2->alpha2=ADRC_Unit[1][13];
  fhan_in2->zeta=ADRC_Unit[1][14];
  fhan_in2->b=ADRC_Unit[1][15];
}



//************************fhan function************************
void Fhan_ADRC(Fhan_Data *fhan_in,float expect_ADRC)// Arrange the ADRC transition process
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;
  x1_delta=fhan_in->x1-expect_ADRC;
  fhan_in->h0=fhan_in->N0*fhan_in->h;
  d=fhan_in->r*fhan_in->h0*fhan_in->h0;
  a0=fhan_in->h0*fhan_in->x2;
  y=x1_delta+a0;
  a1=sqrt(d*(d+8*ABS(y)));
  a2=a0+Sign_ADRC(y)*(a1-d)/2;
  a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
  fhan_in->fh=-fhan_in->r*(a/d)*Fsg_ADRC(a,d)
                  -fhan_in->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));
  fhan_in->x1+=fhan_in->h*fhan_in->x2;
  fhan_in->x2+=fhan_in->h*fhan_in->fh;
}

//***************************fal function******************************
float Fal_ADRC(float e,float alpha,float zeta)
{
    int16 s=0;
    float fal_output=0;
    s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(ABS(e),alpha)*Sign_ADRC(e)*(1-s);
    return fal_output;
}


//***********************ESO********************************
void ESO_ADRC(Fhan_Data *fhan_in)
{
  fhan_in->e=fhan_in->z1-fhan_in->y;
  fhan_in->fe=Fal_ADRC(fhan_in->e,0.5,fhan_in->h);
  fhan_in->fe1=Fal_ADRC(fhan_in->e,0.25,fhan_in->h);


  fhan_in->z1+=fhan_in->h*(fhan_in->z2-fhan_in->beta_01*fhan_in->e);    // Fist observed state update from extended state observer
  fhan_in->z2+=fhan_in->h*(fhan_in->z3-fhan_in->beta_02*fhan_in->fe+fhan_in->b*fhan_in->u);  // Second observed state update from extended state observer
  fhan_in->z3+=fhan_in->h*(-fhan_in->beta_03*fhan_in->fe1);  // Total disturbance term
}

//********************Control Law*************************
void ADRC_Control(Fhan_Data *fhan_in,float expect_ADRC,float feedback_ADRC)
{
  
      Fhan_ADRC(fhan_in,expect_ADRC);

      fhan_in->y=feedback_ADRC;
  
      ESO_ADRC(fhan_in);
      fhan_in->e0+=fhan_in->e1*fhan_in->h;
      fhan_in->e1=fhan_in->x1-fhan_in->z1;
      fhan_in->e2=fhan_in->x2-fhan_in->z2;

      fhan_in->u=Constrain_Float(fhan_in->u0,-200,200); //Nonlinear feedback combiner!
}

