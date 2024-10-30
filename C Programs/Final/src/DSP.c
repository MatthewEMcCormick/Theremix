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
        // Ensure floating-point division
        float normalizedValue = index / ((float)(300 - 1));
        lookUp[index] = tanh(normalizedValue); // Calculate tanh for the normalized value
    }
    lookUp[0] = 1.0f;
}

float lookUpTan(float audioVal)
{
    // Clamp audioVal to the range [0, 1]
    if (audioVal < 0.0f) audioVal = 0.0f;
    if (audioVal > 1.0f) audioVal = 1.0f;

    // Calculate the index based on audioVal
    int index = (int)(audioVal * (300 - 1) + 0.5f); // Round to nearest

    // Clamp index to prevent out-of-bounds access
    if (index < 0) index = 0;
    if (index >= 300) index = 299; // Ensure it doesn't exceed the bounds

    // Return the corresponding tanh value from the LUT
    return lookUp[index];
}
uint16_t saturator(int gainIn, float WD, int gainOut, int audio) //add curve
{
    //int scaleVal = 10;
    float newAudioIn;
    float output;

    //scale input between -1 and 1 for the tanh function
    //makes it more sensative to changes.
    
    //multiply by gain value
    /*if (audio < 1997.5) 
    {
        newAudioIn = (float)audio / 2047.5 * -1;  // Range: [-1, 0]
    } 
    else if(audio > 2097.5)
    {
        newAudioIn = (float)(audio - 2047.5) / 2047.5;  // Range: [0, 1]
    }*/
    newAudioIn = (float)audio / 4095.0f; 
    //process in tan function 
    //output =1.0f / (1.0f + exp(-newAudioIn * WD/100)); //tanh(newAudioIn * WD/100.0f);
    //output = tanh(newAudioIn * WD/100.0f);
    output = lookUpTan(newAudioIn);
    //multiple by outGain value
    /*if (audio < 1997.5) 
    {
        output = (float)(output*-1*2047.5);
    }
    else if(audio > 2097.5)
    {
        output = (output)*2047.5+2047.5;
    }*/
    output = output * 4095;  // Scale back to original range

    // Clamp the output to prevent overflow

    //output = (uint16_t)((output + 1.0f) / 2.0f * 4095);
    
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