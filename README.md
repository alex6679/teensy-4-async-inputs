# teensy-4-async-inputs
## AsyncAudioInput
The high level entry point is the AsyncAudioInput template class (implemented in async_input.h). It inherits from AudioStream and can thus be used as block in the Teensy audio lib. It implements the following pipline:
![pipeline](https://github.com/alex6679/teensy-4-async-inputs/blob/main/imgs/resampling_pipeline.png)
Additionally it performs the following tasks:
- It uses a 'FrequencyMeasurement' object (see below) to retrieve the current sampling frequency of the input data and configures the Resampler accordingly.
- It monitors the input buffer and provides that information to the Resampler. The Resampler then slightly adjust the sampling step in order to prevent buffer over- and underflow.
- It implements the AudioStream-update function and sends out audio blocks of the resampled signal.
### Constructor and member functions
Constructor:  
**AsyncAudioInput(dither, noiseshaping, attenuation, minHalfFilterLength, maxHalfFilterLength);**
dither: triangular shaped dither is added at the transition from 32bit float to 16bit integer if true (default: false)
noiseshaping: noise shaping is applied at the aforementioned transition if true (default: false)
attenuation: target attenuation of the anti-aliasing filter (default: 100dB). The attenuation is not reached if a filter longer than 161 is needed.
minHalfFilterLength: half of the guaranteed resampling filter (internally restricted to 80, default: 20). The filter might be longer if needed to achieve the requested attenuation.
maxHalfFilterLength: Restricts the maximum length of the resampling filter. The maximum half filter length is 80. This parameter can be used to further restrict the length in order to limit the processor usage. 

Member functions:  
**getBufferedTime();**
Returns the buffered time in seconds. The buffered time is the duration of the incoming samples which are not resampled yet. The step width of the resampling algorithm is constantly slightly adjusted to keep the buffered time closely to the target latency. The difference between the target latency and the buffered time is typically smaller than 1 microsecond.

**getInputFrequency();**
Returns the sample rate of incoming data. The incoming sample rate is computed based on the frequency of the isr calls. The computed frequency is low pass filter. -1 is returned, if this filtered frequency is changing at the time of the function call. (The condition for 'not changing' is quite strict and -1 is returned quite often)

**getLastValidFrequency()**
Returns the last stable sample rate of the incoming data. 

**getTargetLantency();**
Returns the target latency in seconds. The latency is the time from the moment a sample is received by the Teensy SPDIF hardware receiver until it is transmitted by the asrc intput. The audio samples arrive at the asrc input in chunks of 32 samples per channel. The target latency consists of these 32 samples + some buffer that is needed to compensate for timing variations.

**getAttenuation();**
Returns the actual achieved attenuation of the anti-aliasing filter. If the input sampling rate is smaller or equal to 44.1kHz, no low pass filtering is needed and zero is returned.

**getHalfFilterLength();**
Returns the half length of the resampling filter. Its complete length is 2*(the returned value)+1.

## TInput
AsyncAudioInputI2Sslave (input_i2s.h/ input_i2s.h.cpp) represents an example implementation of a TInput class. TInput provides the input data (32bit float) to AsyncAudioInput.

## Resampler
Implements the low level resampling algorithm and anti-aliasing filter (Resampler.h/ Resampler.cpp). It initially computes the anti-aliasing filter and step length (e.g. 2.0 for 88.2kHz to 44.1kHz resampling) for a certain input to output sampling frequency. Afterwards the step length is slighlty adjusted if information about error of the sampling step is provided as done by AsyncAudioInput. This feature is not needed if data with a perfectly known sampling frequency is resampled (like wave files). Input and output data is still 32bit float. This class is already part of the Teensy audio library.

## Quantizer
Scales and rounds the data from 32bit floating point to integer (Quantizer.h/ Quantizer.cpp). Optinally dither and noise shaping can be activated. This class is already part of the Teensy audio library.

## FrequencyMeasurement
Implemented in FrequencyMeasurement.h/ FrequencyMeasurement.cpp. This class is not part of the audio pipeline. It is used by AsyncAudioInput to estimate the sampling frequency of the incoming data.
