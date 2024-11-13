#include <math.h>
#include "DSP.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include "global.h"


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
    uint16_t newAudioIn;
    uint16_t output;
    // 41 >> 12 is approximate divide by 100
    // Drive is percentage of wet effect that goes through
    
    newAudioIn = ((audio * WD * 41) >> 12)*(drive * 41) >> 12;
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
    newAudioIn = ((audio * inputGain * 41) >> 12);
    newAudioIn = ((newAudioIn * WD * 41) >> 12);
    if(audio > thres)
    {
        uint16_t over = newAudioIn - thres;
        //output = ((excess >> ratio) + thres);
        postRatio = (over*128)>>ratio;
        output = (((postRatio) + (thres*128))>>7);
    }
    else
    {
        output = newAudioIn;
    }
    output = output + ((audio * (100 - WD) * 41) >> 12);
    output =  (output * outGain * 41) >> 12;
    return output;
}
