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
        uint16_t value = (uint16_t)((float)tanh(step)*4095);
        if(value > 4095)
        {
            value = 4095;
        }
        lookUp[index] = value;
    }
    //lookUp[0] = 1000.0f;
}

uint16_t lookUpTan(int audioVal, int curveFactor)
{
    if (audioVal < 0) 
    {
        audioVal = 0;
    }
    if (audioVal > 4095) 
    {
        audioVal = 4095;
    }
    int newAudio = (audioVal * 3 * curveFactor * 41) >> 12;
    //int index = (int)(audioVal * (300 - 1) + 0.5f);
    int index = ((int)((newAudio * 300 + 2047) / 4096));
    if(index >= 300)
    {
        index = 299;
    }
    return lookUp[index];
}
uint16_t saturator(int drive, int WD, int curve, int audio) //add curve
{
    //int scaleVal = 10;
    uint16_t newAudioIn;
    uint16_t output;
    // 41 >> 12 is approximate divide by 100
    // Drive is percentage of wet effect that goes through
    
    newAudioIn = ((audio * WD * 41) >> 12)*(drive * 41) >> 12;
    /*if(newAudioIn > 4095 - 1861)
    {
        newAudioIn = 4095 - 1861;
    }*/
    //newAudioIn = audio * (drive * 41) >> 12;
    //3 is max curve factor before issues are caused
    output = lookUpTan((int)newAudioIn, curve);
    output = output + ((audio *(100- WD) * 41) >> 12);
    
    if(output > 1241)
    {
        output = 1241;
    }

    return output;
}    
/*
TODO:
1. Implement input and output GAIN
2. Implement Attack
    2.a Possible global variable? similar to last time
    2.b Need to figure out how to reduce division and replace with bitshift for that.
3. Implement Release
*/
uint16_t compressor(int inputGain, int outGain, int WD, int ratio, int thres, int attack, int release, int audio)
{
    uint16_t output = 0;
    uint16_t newAudioIn = 0;
    int postRatio = 0;
    newAudioIn = ((audio * WD * 41) >> 12);
    if(audio > thres)
    {
        uint16_t excess = newAudioIn - thres;
        //output = ((excess >> ratio) + thres);
        postRatio = (excess)>>ratio;
        output = (uint16_t)((postRatio) + (thres));
        /*if(output < thres)
        {
            output = (output * release * 41) >> 12;
        }*/
    }
    else
    {
        output = newAudioIn;
    }
    output = output + ((audio * (100 - WD) * 41) >> 12);
    return output;
}
//Need More effiecient compressor function
/*struct compP compressor(int WD, double threshold, double attack, double release, double ratio, double gainOut, struct compP comp)
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
    
} */