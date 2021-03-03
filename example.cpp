
#include "plotter.h"
#include <Audio.h>
#include "async_input.h"

#define DEBUG
//#define PLOTWAVE

AudioOutputSPDIF3   spdifOut;
#ifdef PLOTWAVE
Plotter plotter(8);  //only plot every 6th sample
#endif

AsyncAudioInputI2Sslave i2sSlaveInput;
AudioConnection          patchCord1(i2sSlaveInput, 0, spdifOut, 0);
AudioConnection          patchCord2(i2sSlaveInput, 1, spdifOut, 1);
#ifdef PLOTWAVE
AudioConnection          patchCord3(i2sSlaveInput, 0, plotter, 0);     
#endif

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
    #ifdef PLOTWAVE
        plotter.activate(true);
    #endif
  #endif
  AudioMemory(15);
 

}

void loop() {
#ifndef PLOTWAVE


	double bufferedTine=i2sSlaveInput.getBufferedTime();
	double targetLatency = i2sSlaveInput.getTargetLantency();
	Serial.print("buffered time [micro seconds]: ");
	Serial.println(bufferedTine*1e6,2);
	Serial.print(", target: ");
	Serial.println(targetLatency*1e6,2);
	
	double pUsageIn=i2sSlaveInput.processorUsage(); 
	Serial.print("processor usage [%]: ");
	Serial.println(pUsageIn);

	double f=i2sSlaveInput.getInputFrequency();
	Serial.print("frequency: ");
	Serial.println(f);
	// Serial.print("Memory max: ");
  	// Serial.println(AudioMemoryUsageMax());
	delay(500);
#endif
}
