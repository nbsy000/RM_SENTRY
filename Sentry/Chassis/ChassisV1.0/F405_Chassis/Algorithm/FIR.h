#ifndef __FIR_H__
#define __FIR_H__

void Fir(float *Input,float *Output);
void LowPass_SetChassis(float* output,float In);
float LowPass_SetWheel(float In,float past);
float LowPass(float Input,float Last_input,float K_LP);

#define ORDER        5                       //½×Êý
#endif
