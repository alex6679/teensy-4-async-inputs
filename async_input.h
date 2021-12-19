/* Audio Library for Teensy 3.X
 * Copyright (c) 2019, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*
 by Alexander Walch
 */
#ifndef async_input_h_
#define async_input_h_
#include "Resampler.h"
#include "Quantizer.h"
#include "biquad.h"
#include "FrequencyMeasurement.h"
#include "AudioStream.h"
#include "Audio.h"
#include "DMAChannel.h"
#include "Arduino.h"
#include <arm_math.h>


template <typename TInput>
class AsyncAudioInput : public AudioStream
{
public:

	#define NOCHANNELS TInput::getNumberOfChannels()

	///@param attenuation target attenuation [dB] of the anti-aliasing filter. Only used if newFs<fs. The attenuation can't be reached if the needed filter length exceeds 2*MAX_FILTER_SAMPLES+1
	///@param minHalfFilterLength If newFs >= fs, the filter length of the resampling filter is 2*minHalfFilterLength+1. If fs y newFs the filter is maybe longer to reach the desired attenuation
	AsyncAudioInput(bool dither=false, bool noiseshaping=false,float attenuation=100, int32_t minHalfFilterLength=20, int32_t maxHalfFilterLength= 80):
		AudioStream(0, NULL),
		_resampler(attenuation, minHalfFilterLength, maxHalfFilterLength)
		{
		_inputFrequency=-1.;
		const float factor = powf(2, 15)-1.f; // to 16 bit audio

		_bufferLPFilter.pCoeffs=new float[5];
		_bufferLPFilter.numStages=1;
		_bufferLPFilter.pState=new float[2];
		getCoefficients(_bufferLPFilter.pCoeffs, BiquadType::LOW_PASS, 0., 5., AUDIO_SAMPLE_RATE_EXACT/AUDIO_BLOCK_SAMPLES, 0.5);
		float* b[NOCHANNELS];
		for (uint8_t i =0 ; i< NOCHANNELS; i++){
			b[i]=&(_buffer[i][0]);
			quantizer[i]= new Quantizer(AUDIO_SAMPLE_RATE);
			quantizer[i]->configure(noiseshaping, dither, factor);
		}
		_noSamplesPerIsr=TInput::getNumberOfSamplesPerIsr();
		_resampleOffset=0;
		__disable_irq();
		_input.setResampleBuffer(b, bufferLength);
		_input.setResampleOffset(_resampleOffset);
		_input.setFrequencyMeasurment(updateFrequMeasure);
		__enable_irq();	
	}
	~AsyncAudioInput(){
		delete [] _bufferLPFilter.pCoeffs;
		delete [] _bufferLPFilter.pState;
		for (uint8_t i =0 ; i< NOCHANNELS; i++){
			delete quantizer[i];
		}
	}

	virtual void update(void)
	{	
		frequMeasure.computeFrequency();	
		bool newConfiguration = configure();				//important configure() before monitorResampleBuffer()
		monitorResampleBuffer(newConfiguration);	//important first call 'monitorResampleBuffer' then 'resample'
		audio_block_t *blocks[NOCHANNELS];
		audio_block_t** b=blocks;
		for (uint8_t i =0; i< NOCHANNELS; i++){
			*b= allocate();
			if(*b==nullptr){
				b=blocks;
				for (uint8_t j=0; j< i; j++){
					release(*b);
					*b++=nullptr;
				}
				break;
			}
			++b;
		}
		if (blocks[0]) {
			int32_t block_offset=0;
			resample(blocks, block_offset);
			if(block_offset < AUDIO_BLOCK_SAMPLES){
				for (uint8_t i =0; i< NOCHANNELS; i++){
					memset(blocks[i]->data+block_offset, 0, (AUDIO_BLOCK_SAMPLES-block_offset)*sizeof(int16_t)); 				
				}
			}
			for (uint8_t i =0; i< NOCHANNELS; i++){
				transmit(blocks[i], i);
				release(blocks[i]);
				blocks[i]=nullptr;
			}
		}
	}
	
	double getTargetLantency() const {
		AudioNoInterrupts();
		double l=_targetLatencyS;
		AudioInterrupts();
		return l ;
	}

	double getAttenuation() const{
		AudioNoInterrupts();
		double a= _resampler.getAttenuation();
		AudioInterrupts();
		return a;
	}

	int32_t getHalfFilterLength() const{
		AudioNoInterrupts();
		int32_t l= _resampler.getHalfFilterLength();
		AudioInterrupts();
		return l;
	}

	double getInputFrequency() const {
		AudioNoInterrupts();
		double f=frequMeasure.getFrequency();
		AudioInterrupts();
		return f;
	}

	double getBufferedTime() const{
		AudioNoInterrupts();
		double n=_bufferedTime;
		AudioInterrupts();
		return n;
	}
private:

	static FrequencyMeasurement frequMeasure;
	Resampler _resampler;
	Quantizer* quantizer[NOCHANNELS];
	TInput _input;
	arm_biquad_cascade_df2T_instance_f32 _bufferLPFilter;
	constexpr static double blockDuration=AUDIO_BLOCK_SAMPLES/AUDIO_SAMPLE_RATE; //[seconds]
	double _maxLatency=2.*blockDuration; 
	constexpr static int32_t bufferLength=(int32_t)( 1.f + ceilf(192000.f/AUDIO_SAMPLE_RATE))*AUDIO_BLOCK_SAMPLES + (int32_t)(ceilf(3.f*blockDuration * 192000.f));	//maximum input sample rate is 192kHz
	float _buffer[NOCHANNELS][bufferLength];
	volatile double _bufferedTime;
	double _inputFrequency;
	int32_t _noSamplesPerIsr;
	int32_t _resampleOffset;
	double _targetLatencyS;	//target latency [seconds]
	uint32_t _updateLast=0;
	

	void resample(audio_block_t **blocks, int32_t& block_offset){
		block_offset=0;
		if(!_resampler.initialized()){
			return;
		}
		__disable_irq();
		int32_t bOffset=_input.getBufferOffset();
		__enable_irq();

		int32_t inputBufferStop = bOffset >= _resampleOffset ? bOffset-_resampleOffset : bufferLength-_resampleOffset;
		if (inputBufferStop==0){
			return;
		}
		
		int32_t processedLength=0;
		float outputs[NOCHANNELS][AUDIO_BLOCK_SAMPLES];
		float* outputsPtr[NOCHANNELS];
		float* inputs[NOCHANNELS];
		for (uint8_t i =0 ; i< NOCHANNELS; i++){
			inputs[i]=_buffer[i]+_resampleOffset;
			outputsPtr[i]=&(outputs[i][0]);
		}
		_resampler.resample<NOCHANNELS>(inputs, inputBufferStop, processedLength, outputsPtr, AUDIO_BLOCK_SAMPLES, block_offset);

		_resampleOffset=(_resampleOffset+processedLength)%bufferLength;

		if (bOffset > _resampleOffset && block_offset< AUDIO_BLOCK_SAMPLES){
			inputBufferStop= bOffset-_resampleOffset;
			int32_t outputLength=AUDIO_BLOCK_SAMPLES-block_offset;
			for (uint8_t i =0 ; i< NOCHANNELS; i++){
				inputs[i]=_buffer[i]+_resampleOffset;
			}
			int32_t outputCount=0;
			_resampler.resample<NOCHANNELS>(inputs, inputBufferStop, processedLength, outputsPtr, outputLength, outputCount);		
			_resampleOffset=(_resampleOffset+processedLength)%bufferLength;
			block_offset+=outputCount;
		}
		for (int32_t i =0; i< NOCHANNELS; i++){
			quantizer[i]->quantize(outputs[i], blocks[i]->data, block_offset);
		}
		__disable_irq();
		_input.setResampleOffset(_resampleOffset);
		__enable_irq();	
	}

	static void updateFrequMeasure(){
		frequMeasure.update();
	}

	bool configure(){
		bool newConfiguration=false;
		const double inputF=frequMeasure.getFrequency();	//returns: -1 ... invalid frequency
		if (inputF > 0.){
			//we got a valid sample frequency
			const double frequDiff=inputF/_inputFrequency-1.;
			if (abs(frequDiff) > 0.01 || !_resampler.initialized()){
				//the new sample frequency differs from the last one -> configure the _resampler again
				_inputFrequency=inputF;		
				_targetLatencyS=max(0.001,(_noSamplesPerIsr*3./2./_inputFrequency));
				_maxLatency=max(2.*blockDuration, 2*_noSamplesPerIsr/_inputFrequency);
				_resampler.configure(_inputFrequency,AUDIO_SAMPLE_RATE);
				newConfiguration=true;
				__disable_irq();
				int32_t bOffset=_input.getBufferOffset();
				_input.setResampleOffset(bOffset);	//we want 'monitorResampleBuffer' to set the correct _resampleOffset!!!
				__enable_irq();
				_resampleOffset=bOffset;
				_updateLast=(uint32_t)(-1);
			}
		}
		return newConfiguration;
	}

	// void monitorResampleBuffer(){
	// 	if(!_resampler.initialized()){
	// 		return;
	// 	}

	// 	__disable_irq();

	// 	int32_t bOffset=_input.getBufferOffset();
	// 	const double dmaOffset=(micros()-frequMeasure.getTimeOfLastUpdate())*1e-6;	//[seconds]

	// 	double inputFrequency=frequMeasure.getLastValidFrequency();
		
	// 	double bTime = _resampleOffset <= bOffset ? (bOffset-_resampleOffset-_resampler.getXPos())/inputFrequency+dmaOffset : (bufferLength-_resampleOffset +bOffset-_resampler.getXPos())/inputFrequency+dmaOffset;	//[seconds]
		
	// 	double diff = bTime- (blockDuration+ _targetLatencyS);	//seconds

	// 	biquad_cascade_df2T<double, arm_biquad_cascade_df2T_instance_f32, float>(&_bufferLPFilter, &diff, &diff, 1);
		
	// 	bool settled=_resampler.addToSampleDiff(diff);
		
	// 	if (bTime > _maxLatency || bTime-dmaOffset<= blockDuration || settled) {	
	// 		double distance=(blockDuration+_targetLatencyS-dmaOffset)*inputFrequency+_resampler.getXPos();
	// 		diff=0.;
	// 		if (distance > bufferLength-_noSamplesPerIsr){
	// 			diff=bufferLength-_noSamplesPerIsr-distance;
	// 			distance=bufferLength-_noSamplesPerIsr;
	// 		}
	// 		if (distance < 0.){
	// 			distance=0.;
	// 			diff=- (blockDuration+ _targetLatencyS);
	// 		}
	// 		double resampleOffsetF=bOffset-distance;
	// 		_resampleOffset=(int32_t)floor(resampleOffsetF);
	// 		_resampler.addToPos(resampleOffsetF-_resampleOffset);
	// 		while (_resampleOffset<0){
	// 			_resampleOffset+=bufferLength;
	// 		}	
	// 		_input.setResampleOffset(_resampleOffset);
	// 		__enable_irq();

	// 		preload(&_bufferLPFilter, (float)diff);
	// 		_resampler.fixStep();		
	// 	}
	// 	else {
	// 		__enable_irq();
	// 	}
	// 	_bufferedTime=_targetLatencyS+diff;
	// }
	void monitorResampleBuffer(bool  newConfiguration){
		__disable_irq();
		uint32_t updateCurrent=ARM_DWT_CYCCNT;
		double dmaOffset=(double)(updateCurrent-frequMeasure.getTimeOfLastUpdate())/(double)F_CPU_ACTUAL;	//[seconds]	
		uint32_t isrDC=frequMeasure.getLastDuration();
		int32_t bOffset=_input.getBufferOffset();
		__enable_irq();	

		if(!_resampler.initialized()){
			return;
		}

		double inputFrequency= AUDIO_SAMPLE_RATE*_resampler.getStep();

		//here we try to compensate for timing variations in the calls of 'update' and 'isr'
		double interrupJitter=0.;
		double isrDiff;
		if (_updateLast != (uint32_t)-1){
			double timeSinceLast = (double)(updateCurrent-_updateLast)/(double)F_CPU_ACTUAL;	//[seconds]
			interrupJitter = blockDuration-timeSinceLast;
			if (interrupJitter > 0.){
				//if interrupJitter > 0 then 'update' was called too fast. This only happens, if the preceding call was too late
				interrupJitter=0.;
			}
			//Until now we 'interrupJitter' captures timing variations of 'update'. But if 'update' and 'isr' were both delayed, we apply a wrong correction.
			//Therefore we try to estimate now if the 'isr' call was also delayed:
			isrDiff= _noSamplesPerIsr / inputFrequency - (double)isrDC/(double)F_CPU_ACTUAL;	// _noSamplesPerIsr / inputFrequency is the exptect time between isr calls
			if (isrDiff < 0.){
				//the isr call was also delayed -> dmaOffset is already quite correct and 'interrupJitter' needs to be corrected
				interrupJitter-=isrDiff;
			}
		}
		_updateLast=updateCurrent;

		double resamplerPos = _resampler.getXPos();
		double bTime = _resampleOffset <= bOffset ?
			(bOffset-(_resampleOffset+resamplerPos))/inputFrequency :
			(bufferLength-(_resampleOffset+resamplerPos) + bOffset )/inputFrequency;	//[seconds]
		double targetOffset = blockDuration + _targetLatencyS - interrupJitter - dmaOffset;
		double diff = bTime-targetOffset;	//seconds
		//double diffOld = diff;

		biquad_cascade_df2T<double, arm_biquad_cascade_df2T_instance_f32, float>(&_bufferLPFilter, &diff, &diff, 1);

		bool settled=false;
		if (!newConfiguration){
			settled=_resampler.updateIncrement(diff);
			//we updated the step widht -> lets update the buffered time, so that we can see if there are really enough samples in the buffer
			bTime*=inputFrequency;
			inputFrequency = AUDIO_SAMPLE_RATE * _resampler.getStep();	//we need to update the inputfrequency since we called 'addToSampleDiff'
			bTime/=inputFrequency;
			//================================================================================================================================
		}
		if (bTime > _maxLatency || bTime <= blockDuration || settled || newConfiguration) {
			if (targetOffset < blockDuration + resamplerPos/inputFrequency ||
				targetOffset > 2.*blockDuration + _targetLatencyS){
				if (dmaOffset < 0. || dmaOffset > _noSamplesPerIsr / inputFrequency){
					dmaOffset =0.5*_noSamplesPerIsr / inputFrequency; //we don't know when the last dma transfer happened. The best we can do is to estimate it.
				}
				targetOffset = blockDuration + _targetLatencyS-  dmaOffset;	
			}
			double targetOffsetSamples = targetOffset*inputFrequency;
			const int32_t targetOffsetI=ceil(targetOffsetSamples);
			__disable_irq();
			_resampleOffset =  targetOffsetI <= bOffset ? bOffset - targetOffsetI : bufferLength -(targetOffsetI-bOffset);
			_input.setResampleOffset(_resampleOffset);
			__enable_irq();
			//correct for the integer resample offset
			_resampler.setPos((double)targetOffsetI-targetOffsetSamples);

			preload(&_bufferLPFilter, 0.);
			_resampler.fixIncrement();	
			diff = 0.;
		}
		
		_bufferedTime=_targetLatencyS+diff;
	}
};
template<typename TInput>
FrequencyMeasurement AsyncAudioInput<TInput>::frequMeasure(TInput::getNumberOfSamplesPerIsr());


#endif