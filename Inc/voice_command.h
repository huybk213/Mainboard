#ifndef __VOICE_COMMAND_H
#define __VOICE_COMMAND_H
#include "main.h"
#include "stdbool.h"

#define NUM_MFCC_FEATURES 		10

typedef enum {
	VAD_NO_CHANGE,
	VAD_DETECT_VOICE,
	VAD_DETECT_SILIENT
} VAD_ResultTypeDef;

/**
 * @brief: 1s is divided into number of frame (in mono).
 * */
#define	NUM_FRAMES_IN_1S		25

/**
 * @brief: 1s (mono) is divided into number of frame.
 * 		  each frame's length if FRAME_LEN (in 16bits)
 * 		  (640 old values)
 * */
#define FRAME_LEN 				640
#define FRAME_SHIFT 			640
#define MFCC_DEC_BITS 			2
#define NUM_OUT_CLASSES 		2
#define NUM_CHANNELS			2
#define SAMPLE_SIZE				2
#define SAMPLE_1_SEC			16000
#define NUM_FRAME_RECORD 		4

void KWS_Init(void);
void Run_KWS(void);
void Classify(void);
void Wake_Up(void);


/**
 * @brief: Copy audio buffer into kws internal process buffer
 * */
void voice_command_fill_buffer(int16_t * data, uint32_t size);

#endif /* __VOICE_COMMAND_H */
