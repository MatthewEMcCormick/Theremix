import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as mPlt
import math


######
# input: numpy array of signal to be processed 
# satPercent: Percentage to scale saturation by
#####
def saturator(input, satPercent):
    scaleVal = 50
    if(satPercent > 0):
        saturatorOutput = np.tanh(input * scaleVal * satPercent)
    else:
        saturatorOutput = input
    return saturatorOutput

def compressor(input, currentOutput, thresLevel, attack, release):
    sampleISegments = [input[x:x+10] for x in range(0, len(input), 10)]
    sampleOSegments = [currentOutput[x:x+10] for x in range(0, len(currentOutput), 10)]
    output = []
    gainR = 4
    currentGR = 1
    switch = 0
    for segI, segO in zip(sampleISegments, sampleOSegments):
        segI[segI == 0] = 1
        gain = 20 * np.log10(np.abs(segO)/np.abs(segI))

        if(np.max(gain) >= thresLevel):
            if(currentGR <= gainR and switch == 0):
                currentGR += gainR/attack
            elif (currentGR > gainR):
                currentGR = gainR
                switch == 1
            elif (switch == 1 and currentGR > 1):
                currentGR -= gainR/release
            else:
                currentGR = 1
            
            outputSeg = segO/currentGR
            output.append(outputSeg)
        # else:
        #     output.append(segO)
    #print(20 * np.log10(np.abs(output)/np.abs(segI)))    
    return np.concatenate(output)
def main():

    #open wav file
    sampleRate, wavData = wavfile.read('WAV/CNote.wav')
    
    #normalize the wav file np array before processing it 
    cycles = 4 # how many sine cycles
    resolution = 1000 # how many datapoints to generate

    length = np.pi * 3 * cycles
    my_wave = 100*np.sin(np.arange(0, length, length / resolution))
    normWavData = 2 * (wavData - np.min(wavData))/(np.max(wavData) - np.min(wavData)) - 1

    #take 0.005 seconds of sample. Small sample so its easier to see in plots
    normWavDataSnip = normWavData[:math.ceil(sampleRate * .005)]
    #print(math.ceil(sampleRate * .005))
    #Process data in saturator 
    satData = saturator(normWavDataSnip, 1)

    #Restore the saturated data back to original constraints
    satData = (satData+1)/2 * (np.max(wavData) - np.min(wavData)) + np.min(wavData)
    compData = compressor(normWavDataSnip,satData,4,3,5)
    
    #plotting outputs 
    # mPlt.figure(1)
    # time = np.linspace(0, len(wavData) / sampleRate, num=len(wavData))
    # mPlt.plot(time, wavData)
    # mPlt.title("Input Wave")

    # mPlt.figure(2)
    
    # time = np.linspace(0, len(satData) / sampleRate, num=len(satData))
    # mPlt.plot(time, satData)
    # mPlt.title("Saturation Output")

    # mPlt.figure(3)
    # time = np.linspace(0, len(compData) / sampleRate, num=len(compData))
    # mPlt.plot(time,compData)
    # mPlt.title("Compressor Output")
    
    sat = saturator(2 * (my_wave - np.min(my_wave))/(np.max(my_wave) - np.min(my_wave)) - 1, .5)
    #sat = saturator(my_wave,1)
    satNorm = (sat+1)/2 * (np.max(my_wave) - np.min(my_wave)) + np.min(my_wave)
    sinComp = compressor(my_wave,my_wave,0,30,50)

    mPlt.figure(4)
    time = np.linspace(0, len(my_wave), num=len(my_wave))
    mPlt.plot(time, my_wave)
    mPlt.title("Input Wave")
    mPlt.figure(5)
    time = np.linspace(0, len(sat), num=len(sat))
    mPlt.plot(time, satNorm)
    mPlt.title("Saturation Output")

    mPlt.figure(6)
    mPlt.plot(my_wave,satNorm)
    mPlt.title("Saturation GAIN")

    mPlt.figure(7)
    time = np.linspace(0, len(sinComp), num=len(sinComp))
    mPlt.plot(time, sinComp)
    mPlt.title("Compressor Output")
    mPlt.show()

if __name__ == '__main__':
    main()