# teensy-4-async-inputs
The high level entry point is the AsyncAudioInput template class (implemented in async_input.h). It inherits from AudioStream and can thus be used as block in the Teensy audio lib.
![pipeline](https://github.com/[alex6679]/teensy-4-async-inputs/blob/main/resampling_pipeline.png?raw=true)
![pipeline](https://github.com/alex6679/teensy-4-async-inputs/blob/main/imgs/resampling_pipeline.png)

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

