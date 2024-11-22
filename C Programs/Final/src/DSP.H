#ifndef DSP_H
#define DSP_H

#include <stdint.h>

uint16_t saturator(int drive, int WD, int curve, int audio);
//struct compP compressor(int WD, double threshold, double attack, double release, double ratio, double gainOut, struct compP comp);
//void initLookUpTan(void);
uint16_t compressor(int inputGain, int outGain, int WD, int ratio, int thres, int audio);
uint16_t lookUpTan(int audioVal, int curveFactor);
struct compP
{
    int activated;
    float currentGR;
    int releaseAct;
    int audio;
};
#endif
 