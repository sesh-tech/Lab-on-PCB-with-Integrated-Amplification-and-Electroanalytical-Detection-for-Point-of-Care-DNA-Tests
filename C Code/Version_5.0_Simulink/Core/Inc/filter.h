
struct LP_FILTER_1_ORDER
{
	float cutoff_freq;
	float sampling_time;

	float reliance_current;
	float reliance_previous;

	float previous_output;
	float output;
};

void setLPF_1Parameters (struct LP_FILTER_1_ORDER *filter, float cutoffFreq, float samplingTime);
void changeLPF_1SamplingTime (struct LP_FILTER_1_ORDER *filter, float samplingTime);
void changeLPF_1CutOffFreq (struct LP_FILTER_1_ORDER *filter, float cutoffFreq);
float getLPF_1Output (struct LP_FILTER_1_ORDER *filter, float sample);
