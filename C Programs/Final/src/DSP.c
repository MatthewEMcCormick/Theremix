#include <math.h>
#include "DSP.h"
#include "stm32f4xx.h"
#include <stdint.h>

/*struct compP
{
    int activated;
    float currentGR;
    int releaseAct;
    int audio;
};*/


uint16_t saturator(int gainIn, float WD, int gainOut, int audio) //add curve
{
    //int scaleVal = 10;
    float newAudioIn;
    float output;

    //scale input between -1 and 1 for the tanh function
    //makes it more sensative to changes.
    
    //multiply by gain value
    if(audio < 2047.5)
    {
        newAudioIn = (float)audio/2047.5*-1
    }
    else
    {
        newAudioIn = ((float)audio-2047.5)/2047.5
    }
    //newAudioIn = (float)audio / 4095.0f ; 
    newAudioIn = (float)audio;
    //process in tan function 
    output = tanh(newAudioIn * WD/100*1000);
    //multiple by outGain value
    output = output* gainOut;
    if(audio < 2047.5)
    {
        output = (float)(output*-1*2047.5)
    }
    else
    {
        output = output*2047.5+2047.5
    }
    //output = (uint16_t)((output + 1.0f) / 2.0f * 4095);
    
    return (uint16_t)output;
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