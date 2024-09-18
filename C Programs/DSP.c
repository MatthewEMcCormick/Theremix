#include <math.h>

void saturator(int gainIn, int WD, int gainOut, double audio)
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

    //scale back to output range

    
}

double gainAdjustment(int mode, double sample, double gain)
{
    //Gain In Adjustment 
    if(mode == 0)
    {
        //add 
    }
    //Gain Out Adjustment 
    else
    {
        //add
    }
}

int dspMain(double audioIn, double WD, double gainIn, double gainOut, double threshold, double attack, double release, double ratio)
{

}