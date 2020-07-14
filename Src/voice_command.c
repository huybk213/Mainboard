#include "voice_command.h"
#include "arm_math.h"
#include "mfcc.h"
#include "dnn.h"
#include "gru.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "rk_serial_protocol_id.h"

#define VOICE_COMMAND_LOCK_RESOURCE(x)       xSemaphoreTake(x, portMAX_DELAY)
#define VOICE_COMMAND_UNLOCK_RESOURCE(x)     xSemaphoreGive(x)

extern void app_error_handler(char * file, int line, int error_code);
extern void Assert_Is_NULL(void* value);

float *audio_buffer_float;

/**
 * @brief saved audio data (in stereo)
 * */
static int16_t *audio_buffer_16;

//static float thresh_hold[8] = { 1.2f, 1.2f, 1.1f, 1.0f, 1.18f, 1.0f, 4.0f, 4.0f };

int16_t mfcc_buffer[250];
int16_t *gru_output;
int16_t dnn_output[2];
float *prediction_history;
float *prediction;

MFCC_TypeDef *mfcc;
DNN_TypeDef *dnn;
GRU_TypeDef *gru;

bool FLAG_wakeup = false;

/**
 * @brief hold result of VAD
*/
VAD_ResultTypeDef vad_result;
bool flag_vad = false; // rising edge: start voice; falling edge: end voice

float time_run  = 100.0f; //0.1s
float time_wait = 10000.0f; //10s
uint32_t stamp1, stamp2, stamp3, stamp4;

void KWS_Init() {
    return;
//    m_protect_voice_resource = xSemaphoreCreateMutex();
//	Assert_Is_NULL(m_protect_voice_resource);
//    VOICE_COMMAND_UNLOCK_RESOURCE(m_protect_voice_resource);
    
    audio_buffer_16 = pvPortMalloc(SAMPLE_1_SEC*2);
    Assert_Is_NULL(audio_buffer_16);
    
    audio_buffer_float = pvPortMalloc(SAMPLE_1_SEC*sizeof(float));
    Assert_Is_NULL(audio_buffer_float);

		gru_output = pvPortMalloc(NUM_OUT_CLASSES * sizeof(int16_t));
		Assert_Is_NULL(gru_output);

		prediction_history = pvPortMalloc(NUM_OUT_CLASSES * NUM_FRAME_RECORD * sizeof(float));
		Assert_Is_NULL(prediction_history);

		prediction = pvPortMalloc(NUM_OUT_CLASSES * sizeof(float));
		Assert_Is_NULL(prediction);

		mfcc = pvPortMalloc(1 * sizeof(MFCC_TypeDef));
		Assert_Is_NULL(mfcc);
		MFCC_Init(mfcc, 10, 640, 2);
    
    dnn = (DNN_TypeDef*)pvPortMalloc(1 * sizeof(DNN_TypeDef));
		Assert_Is_NULL(dnn);
    DNN_Init(dnn);
    
    gru = (GRU_TypeDef*)pvPortMalloc(1 * sizeof(GRU_TypeDef));
    Assert_Is_NULL(gru);
    GRU_Init(gru);
		
		vad_result = VAD_NO_CHANGE;
}

void voice_command_fill_buffer(int16_t * data, uint32_t size)
{
    if (data == NULL || size == 0)
        return;
    if (size > SAMPLE_1_SEC*2)
        size = SAMPLE_1_SEC*2;
    
     // VOICE_COMMAND_LOCK_RESOURCE(m_protect_voice_resource);
    memcpy(audio_buffer_16, data, size);
    // VOICE_COMMAND_UNLOCK_RESOURCE(m_protect_voice_resource);
}

void Run_KWS() {
	// Normalize data
	uint16_t temp_max = 0;

	for (int i = 0; i < SAMPLE_1_SEC; i++) {
		if (temp_max < abs(audio_buffer_16[i])) {
			temp_max = abs(audio_buffer_16[i]);
		}
	}

	// Find max
	for (int i = 0; i < SAMPLE_1_SEC; i++) {
		audio_buffer_float[i] = audio_buffer_16[i] / (float) temp_max; //Normalize
	}
    
	// MFCC
	memset(mfcc_buffer, 0, NUM_FRAMES_IN_1S * NUM_MFCC_FEATURES * 2);
	for (int i = 0; i < NUM_FRAMES_IN_1S; i++) {
		MFCC_Compute(audio_buffer_float + i * FRAME_LEN, mfcc_buffer + i * NUM_MFCC_FEATURES, mfcc);
	}

//	if (FLAG_wakeup == false)
    if (0)
        Wake_Up();
    else 
        Classify();
}


void Classify() {
	Run_NN_GRU(mfcc_buffer, gru_output, gru);
	bool prev_flag_vad = flag_vad;
	flag_vad = gru_output[0]  > gru_output[1];
	
	//Detect edge
	if(prev_flag_vad != flag_vad){
		if(flag_vad){
			//Rising edge
			vad_result = VAD_DETECT_VOICE;
		}else{
			//Falling edge
			vad_result = VAD_DETECT_SILIENT;
		}
	}else{
		vad_result = VAD_NO_CHANGE;
	}
}


void Wake_Up() {
	Run_NN(mfcc_buffer, dnn_output, dnn);
//    float max;

//	if (dnn_output[0] > dnn_output[1])
//    {
//        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);
//    }
//    else 
//        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
            
//		max = (dnn_output[0] + abs(dnn_output[1])) / (dnn_output[0] + dnn_output[1] + 2 * abs(dnn_output[1]));
//		if (max >= (float)0.7) 
//            {
//			time_run = 100;
//			stamp2 = xTaskGetTickCount();
//            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);
////			Send_Command(1);
////            rk_serial_protocol_send_frame(RK_ID_SEND_AUDIO_TO_CLOUD , 0, NULL);
//			FLAG_wakeup = true;
//		}else{
//            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
//        }
}

