int saturator(int gainIn, int WD, int gainOut, int audio);
struct compP compressor(int WD, double threshold, double attack, double release, double ratio, double gainOut, struct compP comp);
struct compP
{
    int activated;
    float currentGR;
    int releaseAct;
    int audio;
};
 