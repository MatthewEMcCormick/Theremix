#include <math.h>
#include "DSP.h"

/*struct compP
{
    int activated;
    float currentGR;
    int releaseAct;
    int audio;
};*/


int saturator(int gainIn, int WD, int gainOut, int audio) //add curve
{
    //int scaleVal = 10;
    double newAudioIn;
    double output;
    //scale input between -1 and 1 (NOT )
    
    //multiply by gain value
    newAudioIn = gainIn * audio; 
    //process in tan function 
    output = tanh(newAudioIn * WD/100);
    //multiple by outGain value
    output = output* gainOut;

    return (int)output;
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