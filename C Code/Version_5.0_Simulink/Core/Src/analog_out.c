	#include "analog_out.h"

	void DAC_Channel_Init 	(struct ANALOGOUT * analogout, DAC_HandleTypeDef * hdac, int channel)		// Function to initialize the channel and create the struct with the ADC and channel number
	{
		analogout->dachandle = hdac;
		analogout->Channel = channel;

		if (analogout->Channel == 1)
			HAL_DAC_Start (analogout->dachandle, DAC_CHANNEL_1);
		if (analogout->Channel == 2)
			HAL_DAC_Start (analogout->dachandle, DAC_CHANNEL_2);
	}

	void DAC_Set_Value 		(struct ANALOGOUT * analogout, uint16_t value)								// Function which returns the particular value of the ADC channel
	{
		if(analogout->Channel == 1)
			 HAL_DAC_SetValue(analogout->dachandle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t) value);
		if(analogout->Channel == 2)
			 HAL_DAC_SetValue(analogout->dachandle, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t) value);
	}

	void DAC_Set_Voltage	(struct ANALOGOUT * analogout, float voltage);
