#include <math.h>

double saturator(int gainIn, int WD, int gainOut, double audio)
{
    int scaleVal = 10;
    double newAudioIn;
    double output;
    //scale input between -1 and 1 (NOT )
    
    //multiply by gain value
    newAudioIn = gainIn * audio; 
    //process in tan function 
    output = tanh(newAudioIn * WD/100);
    //multiple by outGain value
    output = output* gainOut;

    return output;
}



double compressor(double audio, int WD, double threshold, double attack, double release, double ratio, double gainOut, int * Activated, double * currentGR, int * releaseAct)
{
    double output;
    //need to set activated to zero in the intialization call
    if(Activated != 1 && audio > threshold)
    {
         * Activated = 1;
    }
    if (Activated == 1)
    {
        output = audio / *currentGR;
       
        if(* releaseAct == 0)
        { 
            if(*currentGR <= ratio)
            {
                *currentGR += (ratio/attack);
            }
            else if(*currentGR > ratio)
            {
                *currentGR = ratio;
                *releaseAct = 1;
            }
        }
        else
        {
            if(*currentGR > 1)
            {
                *currentGR -= (ratio/release);
            }
            else if(*currentGR < 1)
            {
                *currentGR = 1;
                *releaseAct = 0;
            }
        }
        
    }
    output = output * gainOut;

    return output;
    
}