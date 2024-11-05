#include <math.h>
#include "DSP.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include "global.h"

//extern volatile float lookUp[300];

void initLookUpTan(void)
{
    for (int index = 0; index < 300; index++)
    {
        float step = index / ((float)(300 - 1));
        lookUp[index] = (uint16_t)((float)tanh(step)*4095);
    }
    //lookUp[0] = 1000.0f;
}

uint16_t lookUpTan(int audioVal)
{
    if (audioVal < 0) 
    {
        audioVal = 0;
    }
    if (audioVal > 4095) 
    {
        audioVal = 4095;
    }

    //int index = (int)(audioVal * (300 - 1) + 0.5f);
    int index = (int)((audioVal * 300 + 2047) / 4096);
    return lookUp[index];
}
uint16_t saturator(int gainIn, int WD, int gainOut, int audio) //add curve
{
    //int scaleVal = 10;
    uint16_t newAudioIn;
    uint16_t output;

    //scale input between -1 and 1 for the tanh function
    newAudioIn = ((audio * WD * 41) >> 12);
    //process in tan function 
    //output =1.0f / (1.0f + exp(-newAudioIn * WD/100)); //tanh(newAudioIn * WD/100.0f);
    //output = tanh(newAudioIn * WD/100.0f);
    output = lookUpTan((int)newAudioIn);
    output = output + ((audio *(100- WD) * 41) >> 12);

    
    return output;
}    
//add gain IN paramter
struct compP compressor(int WD, double threshold, double attack, double release, double ratio, double gainOut, struct compP comp)
{
    int output;
    //need to set activated to zero in the intialization call
    if(comp.activated != 1 && comp.audio > threshold)
    {
         comp.activated = 1;
    }
    if (comp.activated == 1)
    {
        output = comp.audio / comp.currentGR;
       
        if(comp.releaseAct == 0)
        { 
            if(comp.currentGR <= ratio)
            {
                comp.currentGR += (ratio/attack);
            }
            else if(comp.currentGR > ratio)
            {
                comp.currentGR = ratio;
                comp.releaseAct = 1;
            }
        }
        else
        {
            if(comp.currentGR > 1)
            {
                comp.currentGR -= (ratio/release);
            }
            else if(comp.currentGR < 1)
            {
                comp.currentGR= 1;
                comp.releaseAct = 0;
            }
        }
        
    }
    output = output * gainOut;
    comp.audio = output;
    return comp;
    
} 