#ifndef DSP_H
#define DSP_H

#include <stdint.h>

uint16_t saturator(int gainIn, int WD, int gainOut, int audio);
struct compP compressor(int WD, double threshold, double attack, double release, double ratio, double gainOut, struct compP comp);
//void initLookUpTan(void);
uint16_t lookUpTan(int audioVal);
struct compP
{
    int activated;
    float currentGR;
    int releaseAct;
    int audio;
};
#endif
 