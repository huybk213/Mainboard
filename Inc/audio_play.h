#ifndef INCLUDE_AUDIO_PLAY_H_
#define INCLUDE_AUDIO_PLAY_H_

#include "constant.h"
#include "stdint.h"
#include "stdbool.h"
//#include "stm32f4xx_hal_spi.h"

#define AUDIO_FREQ_8K   8000
#define AUDIO_FREQ_11K  11025
#define AUDIO_FREQ_16K  16000
#define AUDIO_FREQ_22K  22050
#define AUDIO_FREQ_32K  32000
#define AUDIO_FREQ_44K  44100
#define AUDIO_FREQ_48K  48000
#define AUDIO_FREQ_96K  96000
#define AUDIO_FREQ_192K 192000

#define MAX_NUM_QUEUED_AUDIO_SECTIONS 12

typedef enum {
	AUDIO_STATE_IDLE = 0,
	AUDIO_STATE_INIT,
	AUDIO_STATE_PLAYING,
	AUDIO_STATE_PAUSE,
} AUDIO_PLAY_StateTypeDef;

typedef enum{
  AUDIO_TRANSFER_SERVER_IDLE = 0,
  AUDIO_TRANSFER_SERVER_WAIT,
} AUDIO_TRANSFER_SERVER_StateTypeDef;

/**
 * @brief The maximum length (in byte) of audio saved on a element in the audio queue
 * */
#define MAX_AUDIO_LENGTH_PER_ELEMENT_IN_QUEUE   4000

typedef struct
{
  uint8_t* data;
  uint32_t size; //size in bytes
  bool is_in_ram; //if true, program need frees data after using. false: in flash
} Common_Data_TypeDef;

typedef struct Audio_Node {
    int16_t* data;
    struct Audio_Node* next;
    struct Audio_Node* prev;
    uint16_t data_size;
    bool has_data;
} Audio_Node;

/**
 * @brief Change status of recording
 * @param status: true: start recording, false: stop recording
 * */
void Audio_Set_Recording(bool status);

/**
 * @brief play a audio file (raw data)
 * @param audio_start: the starting point of audio
 * @param audio_size: lenght of the audio
*/
void Audio_Play(uint8_t* audio_start, uint32_t audio_size);

/**
 * @brief Init audio codec
 * @param volume volume for audio
 * @param freq frequency
 * @param request_More_Audio_Callback callback function which called when more audio data is needed
 * @param providing_Audio_Data_Func callback function which called when more audio data is needed
*/
void Audio_Play_Init(uint32_t volume, uint32_t freq,
		void (*request_More_Audio_Callback)(void *arg));

void Audio_Set_Requesting_More_Audio_Callback(void (*more_Audio_Func)(void *arg));

void Audio_Fill_Data_From_Main(uint8_t *audio, uint32_t size);

/**
 * @brief Set the size of played audio
 * @param size: the size of audio
*/
void Audio_Set_Lengh(uint32_t size);

/**
 * @brief Set the current position of a audio
 * @param offset: the current position
*/
void Audio_Set_Current_Offset(uint32_t offset);

/**
 * @brief Get the total lengh of the audio which is been played
*/
uint32_t Audio_Get_Total_Length_Played_Audio(void);

/**
 * @brief Get the current position of the played audio
*/
uint32_t Audio_Get_Current_Offset(void);

/**
 * @brief push a piece of audio to the FIFO queue
 * @param audio: the starting point of audio
 * @param size: lenght of the piece of audio
*/
void Audio_Push_Queue(uint8_t *audio, uint32_t size);

bool Audio_Is_Full_Queue(void);

/**
 * @brief Stop playing a audio
*/
void Audio_Play_Stop(void);

/**
 * @brief Start playing audio after pushing data to the internal queue (by calling Audio_Push_Queue function)
*/
AUDIO_ErrorTypeDef Audio_Start(void);

/**
 * @brief Pause playing audio
*/
void Audio_Pause(void);

/**
 * @brief Resume playing audio which is paused
*/
void Audio_Resume(void);

/**
 * @brief Get the current status of playing a audio
*/
AUDIO_PLAY_StateTypeDef Audio_Get_Playing_State(void);

/**
 * @brief audio sent by esp32 over SPI DMA.
 *       audio will enqueue to the queue to be played
 * @param hspi: exptect spi2
*/
void Audio_Handle_Received_Audio(SPI_HandleTypeDef *hspi);

#endif /* INCLUDE_AUDIO_PLAY_H_ */
