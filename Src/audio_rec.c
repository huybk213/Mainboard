#include "main.h"
#include <stdio.h>
#include "string.h"
#include "audio_rec.h"
#include "constant.h"

extern void app_error_handler(int error_code);

void Audio_Record ()
{
	/*BSP_AUDIO_IN_Init((uint32_t)voice_config->sample_rate, voice_config->bit_resolution, voice_config->num_channel);
	BSP_AUDIO_IN_Record(internal_buffer, voice_config->block_size);
	BSP_AUDIO_IN_Stop(0);*/
}

/**
  * @brief  Audio IN Error callback function.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_Error_CallBack(void)
{

  app_error_handler((int)ERR_AUDIO_REC);
}
