#include "main.h"
#include <stdio.h>
#include "audio_play.h"
#include "constant.h"
#include "log.h"
#include "esp32.h"
#include "cQueue.h"
#include "limits.h"
#if 0
#include "es8311.h"
#endif
//#include "stm32429i_mainboard_audio.h"
#include "voice_command.h"
//#include "Resource/audio_please_repeat.h"
#include "string.h"

/**
 * Buffer for playing audio
 * 4076 max size of audio (mono type) transmitted by SPI
 * x2 to become stereo type
 * x2 to play half and full audio
 * old value: MAX_AUDIO_LENGTH_PER_ELEMENT_IN_QUEUE*2*2
 */
#define AUDIO_BUFFER_SIZE       4000*2*2

typedef enum {
	BUFFER_OFFSET_NONE = 0,
	BUFFER_OFFSET_HALF,
	BUFFER_OFFSEt_HALF_FILLED,
	BUFFER_OFFSET_FULL,
	BUFFER_OFFSEt_FULL_FILLED,
} BUFFER_StateTypeDef;

typedef struct {
	uint8_t buff[AUDIO_BUFFER_SIZE]; 		//A ring buffer to be used to play audio
	uint8_t record_buff[AUDIO_BUFFER_SIZE]; //A ring buffer to which audio saved
	uint32_t total_size;             		//Total size of audio which will be played
	uint32_t current_offset;         		//Current offset of the audio (queued audio is included)
	BUFFER_StateTypeDef state;
} AUDIO_BufferTypeDef;


extern void app_error_handler(int error_code);
extern void Assert_Is_NULL(void* value);

static const char *TAG = "AUDIO_PLAY";
static void (*Request_More_Audio_Callback)(void *arg);

static AUDIO_BufferTypeDef buffer_ctl;
Queue_t *audio_queue = NULL;
static AUDIO_PLAY_StateTypeDef audio_state;
uint16_t *audio_tracking;

extern I2S_HandleTypeDef        haudio_i2s;
extern int16_t *audio_buffer_in;
Audio_Node* audio_ring_buffer_in;
extern int16_t *audio_buffer_3s;
int16_t *audio_buffer_3s_run;

extern int16_t *audio_buffer_16;
extern float *audio_buffer_float;
static volatile bool recording_is_enable;

Audio_Node* Audio_Create_Node(Audio_Node* prev, size_t audio_size, int16_t* audio_start_addr);
Audio_Node* Audio_Create_Ring_Buff(size_t num_element, size_t audio_size_on_each_node, int16_t* audio_start_addr);
void Audio_Destroy_Ring_Buff(Audio_Node* node);


static void Audio_Mono_To_Stereo(Common_Data_TypeDef *data, uint16_t *audio_start_address) {
	int loop_no = data->size/2;
	for (int i = 0; i < loop_no; ++i) {
		*(audio_start_address + i * 2)     = *((uint16_t*)data->data + i);
		*(audio_start_address + i * 2 + 1) = *((uint16_t*)data->data + i);
	}
}

/**
 * @brief: get mono from stereo
 * @param dest : the destination to which data is copied
 * @param sourc: the source from which data is gotten
 * @param size : size of the audio source (stereo)
 * */
void Audio_Get_Mono(int16_t *dest, int16_t *sourc, uint32_t size) {
	//memcpy(dest, sourc, size/2);
	for (int i = 0; i < size / 2; ++i) {
		*(dest + i) = *(sourc + i * 2);
	}
}

//void Audio_fill_stereo(uint16_t *dest, uint16_t *sourc, uint32_t size) {
//	for (int i = 0; i < size; ++i) {
//		if ((uint8_t*) (sourc + i) >= (uint8_t*) ___Resource_please_repeat_pcm	+ ___Resource_please_repeat_pcm_size) {
//			return;
//		}
//		*(dest + i * 2) = *(sourc + i);
//		*(dest + i * 2 + 1) = *(sourc + i);
//	}
//}

void Audio_Set_Recording(bool status){
	recording_is_enable = status;
}

/**
 * @brief fill audio data for half the audio ring
 * the length is fixed as AUDIO_BUFFER_SIZE/2
 */
static bool Audio_Fill_Data(uint16_t *audio_start_address) {
	Common_Data_TypeDef *audio = NULL;
	//TODO mutex here
	if (q_getCount(audio_queue) <= 0) {
		return false;
	}
	if (q_pop(audio_queue, &audio)) {
    	memset(audio_start_address, 0, AUDIO_BUFFER_SIZE/2);
		Audio_Mono_To_Stereo(audio, audio_start_address);
		if(audio->is_in_ram)
			free(audio->data);
		free(audio);
		return true;
	} else {
		return false;
	}
}

void Audio_Play(uint8_t* audio_start, uint32_t audio_size) {
	uint8_t *audio = audio_start;
	uint8_t *audio_end = (uint8_t*) audio_start + audio_size;
	uint32_t size = MAX_AUDIO_LENGTH_PER_ELEMENT_IN_QUEUE;

	Audio_Set_Lengh(audio_size);
	Audio_Set_Current_Offset(audio_size); //No more data

	//Push all audio to the queue
	while (1) {
		if ((audio_end - audio) < size) {
			size = audio_end - audio;
		}
		Common_Data_TypeDef *data = calloc(1, sizeof(Common_Data_TypeDef));
		Assert_Is_NULL(data);
		data->is_in_ram = false; //in flash
		data->size = size;
		data->data = audio;
		if (!q_push(audio_queue, &data)) {
			free(data);
			data = NULL;
			break;
		}else{
			audio += size;
			if (audio >= audio_end) {
				break;
			}
		}
	}


//	BSP_AUDIO_OUT_HalfTransfer_CallBack();
	//Play audio from the queue
	Audio_Start();
}

void Audio_Play_Init(uint32_t volume, uint32_t freq, void (*more_Audio_Func)(void *arg)) {
#if RK_CODEC_ENABLE
//	if (BSP_AUDIO_OUT_Init(volume, freq, I2S_DATAFORMAT_16B) == 0) {
//		RK_LOGD(TAG, "Audio codec: OK \n");
//	} else {
//		RK_LOGD(TAG, "Audio codec: Fail \n");
//	}
	BSP_AUDIO_IN_Init(freq, I2S_DATAFORMAT_16B);

	BSP_AUDIO_OUT_SetVolume(volume);

	Request_More_Audio_Callback = more_Audio_Func;
	audio_queue = calloc(1, sizeof(Queue_t));
	Assert_Is_NULL(audio_queue);

	q_init(audio_queue, sizeof(Common_Data_TypeDef*), MAX_NUM_QUEUED_AUDIO_SECTIONS, FIFO, false);
	buffer_ctl.total_size = INT_MAX;

	audio_buffer_3s = calloc(SAMPLE_1_SEC * 3, sizeof(int16_t));
	Assert_Is_NULL(audio_buffer_3s);
	audio_buffer_3s_run = audio_buffer_3s;

	//audio_buffer_16 = audio_buffer_3s + SAMPLE_1_SEC * 2;

	audio_buffer_in = calloc(FRAME_LEN * NUM_CHANNELS * 2, sizeof(int16_t));
	Assert_Is_NULL(audio_buffer_in);
	//audio_buffer_float = calloc(SAMPLE_1_SEC, sizeof(float));
	//Assert_Is_NULL(audio_buffer_float);

	//audio_ring_buffer_in = Audio_Create_Ring_Buff(12, FRAME_LEN, (int16_t*)SDRAM_AUDIO_SAVING_START_ADDR);
#endif
}

void Audio_Set_Requesting_More_Audio_Callback(void (*more_Audio_Func)(void *arg)) {
	Request_More_Audio_Callback = more_Audio_Func;
}

void Audio_Play_Stop() {
//	BSP_AUDIO_OUT_Stop();
//	audio_state = AUDIO_STATE_IDLE;
//	Common_Data_TypeDef *value = NULL;
//	while (q_pop(audio_queue, &value)) {
//		free(value->data);
//		free(value);
//		value = NULL;
//	}
//	buffer_ctl.total_size = INT_MAX;
}

void Audio_Play_Stop_Test() {
//	BSP_AUDIO_OUT_Stop();
//	audio_state = AUDIO_STATE_IDLE;
//	Common_Data_TypeDef *value = NULL;
//	while (q_pop(audio_queue, &value)) {
////		free(value->data);
//		free(value);
//		value = NULL;
//	}
//	buffer_ctl.total_size = INT_MAX;
}

void Audio_Set_Lengh(uint32_t size){
  buffer_ctl.total_size = size;
}

void Audio_Set_Current_Offset(uint32_t offset){
  buffer_ctl.current_offset = offset;
}

uint32_t Audio_Get_Total_Length_Played_Audio(){
  return buffer_ctl.total_size;
}

uint32_t Audio_Get_Current_Offset(){
  return buffer_ctl.current_offset;
}

void Audio_Push_Queue(uint8_t *audio, uint32_t size) {
	//TODO mutex here
	Common_Data_TypeDef *data = calloc(1, sizeof(Common_Data_TypeDef));
	Assert_Is_NULL(data);
	data->is_in_ram = true;
	data->data = calloc(size, sizeof(uint8_t));
	Assert_Is_NULL(data->data);

	data->size = size;
	memcpy(data->data, audio, size);
	if (!q_push(audio_queue, &data)) {
		free(data->data);
		free(data);
		data->data = NULL;
		data = NULL;
		app_error_handler(1);
	}
}

bool Audio_Is_Full_Queue(){
	return q_isFull(audio_queue);
}

AUDIO_ErrorTypeDef Audio_Start() {
//	RK_LOGI(TAG, "Audio start\r\n");
////	BSP_AUDIO_IN_DeInit();
////	uint8_t err = BSP_AUDIO_OUT_Init(100, 16000, I2S_DATAFORMAT_16B);

//	if (err != 0) {
//		app_error_handler(err);
//	}

//	if(buffer_ctl.state != BUFFER_OFFSET_NONE){
//		RK_LOGI(TAG, "In [%s] - Audio stop\r\n", __FUNCTION__);
//		BSP_AUDIO_OUT_Stop();
//	}
//	buffer_ctl.state = BUFFER_OFFSET_NONE;
//	memset(buffer_ctl.buff, 0, AUDIO_BUFFER_SIZE);
//	HAL_Delay(500);

//	BSP_AUDIO_OUT_SetVolume(100);
//	BSP_AUDIO_OUT_Play((uint16_t*) buffer_ctl.buff, AUDIO_BUFFER_SIZE);
//	audio_state = AUDIO_STATE_PLAYING;
	return AUDIO_ERROR_NONE;
}

//void Audio_Pause(){
//	audio_state = AUDIO_STATE_PAUSE;
////  	BSP_AUDIO_OUT_Pause();
//}

//void Audio_Resume(){
////	audio_state = AUDIO_STATE_PLAYING;
////	Audio_Fill_Data((uint16_t*) ((buffer_ctl.state == BUFFER_OFFSET_HALF) ? buffer_ctl.buff : (buffer_ctl.buff + AUDIO_BUFFER_SIZE / 2)));
////	BSP_AUDIO_OUT_Resume();
//}

//AUDIO_PLAY_StateTypeDef Audio_Get_Playing_State(){
//	return audio_state;
//}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {
//	RK_LOGI(TAG, "Audio in cplt callback\r\n");
//	if(!recording_is_enable){
//		return;
//	}
//	memmove(audio_buffer_3s, audio_buffer_3s + FRAME_LEN, FRAME_LEN * 2 * (NUM_FRAMES_IN_1S * 3 - 1));
//	Audio_Get_Mono(audio_buffer_3s + FRAME_LEN*(NUM_FRAMES_IN_1S*3 - 1), audio_buffer_in + FRAME_LEN*2, FRAME_LEN*NUM_CHANNELS);

//	//Audio_Get_Mono(audio_buffer_3s_run, audio_buffer_in + FRAME_LEN*2, FRAME_LEN*NUM_CHANNELS);
//	audio_buffer_3s_run += FRAME_LEN;
//	if(audio_buffer_3s_run >= audio_buffer_3s + SAMPLE_1_SEC * 3){
//		audio_buffer_3s_run = audio_buffer_3s;
//	}

////	Audio_Get_Mono(audio_ring_buffer_in->data, audio_buffer_in + FRAME_LEN*2, FRAME_LEN*NUM_CHANNELS);
////	audio_ring_buffer_in = audio_ring_buffer_in->next;
//	return;
//}

//void BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
//	RK_LOGI(TAG, "Audio in 1/2 callback\r\n");
//	if(!recording_is_enable){
//			return;
//	}
//	memmove(audio_buffer_3s, audio_buffer_3s + FRAME_LEN, FRAME_LEN * 2 * (NUM_FRAMES_IN_1S * 3 - 1));
//	Audio_Get_Mono(audio_buffer_3s + FRAME_LEN*(NUM_FRAMES_IN_1S*3 - 1), audio_buffer_in, FRAME_LEN*NUM_CHANNELS);

//	//Audio_Get_Mono(audio_buffer_3s_run, audio_buffer_in, FRAME_LEN*NUM_CHANNELS);
//	audio_buffer_3s_run += FRAME_LEN;
//	if(audio_buffer_3s_run >= audio_buffer_3s + SAMPLE_1_SEC * 3){
//		audio_buffer_3s_run = audio_buffer_3s;
//	}


////	Audio_Get_Mono(audio_ring_buffer_in->data, audio_buffer_in, FRAME_LEN*NUM_CHANNELS);
////	audio_ring_buffer_in = audio_ring_buffer_in->next;
	return;
}

/**
 * @brief  Manages the DMA Half Transfer complete event.
 * @param  None
 * @retval None
 */
static volatile uint32_t done_cnt = 0;
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
//	buffer_ctl.state = BUFFER_OFFSET_HALF;
//	if (audio_state == AUDIO_STATE_PLAYING) {
//		if (Audio_Fill_Data((uint16_t*) buffer_ctl.buff)) {
//			//SCB_CleanDCache_by_Addr((uint32_t*) buffer_ctl.buff, AUDIO_BUFFER_SIZE / 2);
//		} else if (buffer_ctl.total_size >= buffer_ctl.current_offset) {
//			//Waiting for more data
//			RK_LOGI(TAG, "Audio pause\r\n");
//			BSP_AUDIO_OUT_Pause();
//			audio_state = AUDIO_STATE_PAUSE;
//		} else {
//			RK_LOGI(TAG, "In BSP_AUDIO_OUT_HalfTransfer_CallBack - Audio stop\r\n");
//			BSP_AUDIO_OUT_Stop();
//			audio_state = AUDIO_STATE_IDLE;
//			buffer_ctl.state = BUFFER_OFFSET_NONE;
//		}
//	}
//	else
//	{
//		RK_LOGI(TAG, "Invalid audio starte %d\r\n", audio_state);
//	}
//	

//	if(buffer_ctl.total_size > buffer_ctl.current_offset && !q_isFull(audio_queue)){
//		if(Request_More_Audio_Callback != NULL){
//			Request_More_Audio_Callback(NULL);
//		}
//	}
}

/**
 * @brief  Manages the full Transfer complete event.
 * @param  None
 * @retval None
 */
static volatile uint32_t request_more_cnt = 0;
void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
//	RK_LOGI(TAG, "Audio out cb\r\n");
//	buffer_ctl.state = BUFFER_OFFSET_FULL;

//	if (audio_state == AUDIO_STATE_PLAYING) {
//		if (Audio_Fill_Data((uint16_t*) (buffer_ctl.buff + AUDIO_BUFFER_SIZE / 2))) {
//			RK_LOGI(TAG, "Audio fill data\r\n");
//			//SCB_CleanDCache_by_Addr((uint32_t*) (buffer_ctl.buff + AUDIO_BUFFER_SIZE / 2), AUDIO_BUFFER_SIZE / 2);
//			//TODO: check here
//		} else if (buffer_ctl.total_size >= buffer_ctl.current_offset) {
//			//Waiting for more data
//			RK_LOGI(TAG, "Audio more data\r\n");
//			BSP_AUDIO_OUT_Pause();
//			audio_state = AUDIO_STATE_PAUSE;
//		} else {
//			RK_LOGI(TAG, "Audio stop\r\n");
//			BSP_AUDIO_OUT_Stop();
//			audio_state = AUDIO_STATE_IDLE;
//			buffer_ctl.state = BUFFER_OFFSET_NONE;
//		}
//	}
//	else
//	{
//		RK_LOGI(TAG, "Invalid audio state %d\r\n", AUDIO_STATE_PLAYING);
//	}
//	
//	if(buffer_ctl.total_size > buffer_ctl.current_offset && !q_isFull(audio_queue)){
//		if(Request_More_Audio_Callback != NULL){
//			Request_More_Audio_Callback(NULL);
//			request_more_cnt++;
//			if (request_more_cnt > 68)
//			{
//				volatile uint8_t dbg = 1;
//			}
//		}
//	}
//	else
//	{
//		done_cnt++;
//	}
}

void Audio_Fill_Data_From_Main(uint8_t *audio, uint32_t size){
	uint8_t *address = (buffer_ctl.state = BUFFER_OFFSET_HALF) ? buffer_ctl.buff : (buffer_ctl.buff + AUDIO_BUFFER_SIZE / 2);
	//RK_LOG_HEXDUMP(TAG, audio, 16, RK_LOG_INFO);
	if (audio_state == AUDIO_STATE_PLAYING) {
		memset(address, 0, AUDIO_BUFFER_SIZE/2);
		Common_Data_TypeDef compact_audio;
		compact_audio.data = audio;
		compact_audio.size = size;
		Audio_Mono_To_Stereo(&compact_audio, (uint16_t*)address);
		buffer_ctl.state = (buffer_ctl.state = BUFFER_OFFSET_HALF) ? BUFFER_OFFSEt_HALF_FILLED : BUFFER_OFFSEt_HALF_FILLED;
	}
}

/**
 * @brief  Manages the DMA FIFO error event.
 * @param  None
 * @retval None
 */
void BSP_AUDIO_OUT_Error_CallBack(void) {
	app_error_handler((int) ERR_AUDIO_OUT);
}

void Audio_Handle_Received_Audio(SPI_HandleTypeDef *hspi){
//	if((*hspi->pRxBuffPtr == CMD_CHATBOX_AUDIO) || (*hspi->pRxBuffPtr == CMD_CHATBOX_RESPONSE)){
//  		Audio_Push_Queue(hspi->pRxBuffPtr + HEADER_OFFSET_RESPONSE_DATA, *(uint32_t*)(hspi->pRxBuffPtr + HEADER_OFFSET_RESPONSE_DATA_SIZE));
//		buffer_ctl.total_size = *(uint32_t*)(hspi->pRxBuffPtr + HEADER_OFFSET_RESPONSE_TOTAL_SIZE);
//		buffer_ctl.current_offset = *(uint32_t*)(hspi->pRxBuffPtr + HEADER_OFFSET_RESPONSE_CURRENT_OFFSET) +
//									*(uint32_t*)(hspi->pRxBuffPtr + HEADER_OFFSET_RESPONSE_DATA_SIZE);

//		if(audio_state == AUDIO_STATE_PAUSE){
//			audio_state = AUDIO_STATE_PLAYING;
//			BSP_AUDIO_OUT_Resume();
//		}
//	}
//	else
//	{
//		RK_LOGE(TAG, "Recv audio failed %d\r\n", *hspi->pRxBuffPtr);
//	}
}


/* Create ring buffer*/

/**
 * @brief create a node in the ring buffer
 *
 * @param audio_size: number of sample (in 2 bytes)
 * */
Audio_Node* Audio_Create_Node(Audio_Node* prev, size_t audio_size, int16_t* audio_start_addr) {
    Audio_Node* result = (Audio_Node*)calloc(1, sizeof(Audio_Node));
    Assert_Is_NULL(result);

    result->data = calloc(audio_size, sizeof(int16_t));//audio_start_addr;
    Assert_Is_NULL(result->data);

    result->data_size = audio_size;
    memset(result->data, 0, result->data_size*2);
    if (prev != NULL) {
        result->prev = prev;
        prev->next = result;
    }
    return result;
}

Audio_Node* Audio_Create_Ring_Buff(size_t num_element, size_t audio_size_on_each_node, int16_t* audio_start_addr) {
    if (num_element < 1 || audio_size_on_each_node <= 0)
        return NULL;
    Audio_Node* root = Audio_Create_Node(NULL, audio_size_on_each_node, audio_start_addr);
    Audio_Node* node = root;
    for (int i = 1; i < num_element; ++i) {
        node = Audio_Create_Node(node, audio_size_on_each_node, audio_start_addr + i*audio_size_on_each_node);
    }
    node->next = root;
    root->prev = node;
    return root;
}

void Audio_Destroy_Ring_Buff(Audio_Node* node) {
    node->prev->next = NULL;
    //TODO no need
}
