/*Name: Patrice Gama
  Date: 2020*/


#ifndef _ADRC_H_
#define _ADRC_H_

// typedef struct
// {

// float x1;
// float x2;
// float r;
// float h;
// uint16 N0;

// float h0;
// float fh;
// float z1;
// float z2;
// float z3;
// float e;
// float y;
// float fe;
// float fe1;
// float beta_01;
// float beta_02;
// float beta_03;
// float b;
// float e0;
// float e1;
// float e2;
// float u0;
// float u;
// float b0;

// }Fhan_Data;

void ADRC_Init(Fhan_Data *fhan_In1,Fhan_Data *fhan_In2);
void Fhan_ADRC(Fhan_Data *fhan_In,float expect_ADRC);
void ADRC_Control(Fhan_Data *fhan_In,float expect_ADRC,float feedback);

extern Fhan_Data ADRC_Pitch_Controller,ADRC_Roll_Controller;
#endif

