
	#include <math.h>
	#include <stdlib.h>

	#include "filter.h"

	
	void setLPF_1Parameters (struct LP_FILTER_1_ORDER *filter, float cutoffFreq, float samplingTime)
	{

		filter->cutoff_freq = cutoffFreq;
		filter->sampling_time = samplingTime;

		filter->reliance_current = 1 - expf(-filter->sampling_time/filter->cutoff_freq);
		filter->reliance_previous = expf(-filter->sampling_time/filter->cutoff_freq);

		filter->previous_output = 0;
		filter->output = 0;

	}

	void changeLPF_1SamplingTime (struct LP_FILTER_1_ORDER *filter, float samplingTime)
	{
		setLPF_1Parameters (filter, filter->cutoff_freq, samplingTime);
	}

	void changeLPF_1CutOffFreq (struct LP_FILTER_1_ORDER *filter, float cutoffFreq)
	{
		setLPF_1Parameters (filter, cutoffFreq, filter->sampling_time);
	}

	float getLPF_1Output (struct LP_FILTER_1_ORDER *filter, float sample)
	{

		filter->output = filter->reliance_current*sample + filter->reliance_previous*filter->previous_output;

		filter->previous_output = filter->output;

		return filter->output;
	}
