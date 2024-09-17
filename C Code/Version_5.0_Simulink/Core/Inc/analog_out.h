	#include <stdlib.h>
	#include "main.h"

	struct ANALOGOUT							// Structure to define the analog input channels and their data
		{
			DAC_HandleTypeDef * dachandle;		// ADC which the channel is connected to
			int Channel;						// Channel of the pin, if channel 5 and 6 are connected and the rank of 5 > rank of 6, then channel of 5 is 1 and channel of 6 is 2
		};

	void DAC_Channel_Init 	(struct ANALOGOUT * analogout, DAC_HandleTypeDef * hdac, int channel);		// Function to initialize the channel and create the struct with the ADC and channel number
	void DAC_Set_Value 		(struct ANALOGOUT * analogout, uint16_t value);								// Function which returns the particular value of the ADC channel
	void DAC_Set_Voltage	(struct ANALOGOUT * analogout, float voltage);
