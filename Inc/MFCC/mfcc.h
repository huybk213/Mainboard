#ifndef __MFCC_H_
#define __MFCC_H_

#include "arm_math.h"
#include "string.h"
#include <stdlib.h>
#include "float.h"

#define SAMP_FREQ 16000
#define NUM_FBANK_BINS 40
#define MEL_LOW_FREQ 20
#define MEL_HIGH_FREQ 4000
#define M_2PI 6.283185307179586476925286766559005f

typedef struct{
  int num_mfcc_features;
  int frame_len;
  int frame_len_padded;
  int mfcc_dec_bits;
  float * frame;
  float * buffer;
  float * mel_energies;
  float * window_func;
  int32_t * fbank_filter_first;
  int32_t * fbank_filter_last;
  float ** mel_fbank;
  float * dct_matrix;
  arm_rfft_fast_instance_f32 * rfft;
}MFCC_TypeDef;

void MFCC_Init(MFCC_TypeDef *mfcc,int Num_MFCC_Features, int Frame_Len, int MFCC_Dec_Bits);

void MFCC_Compute(const float * audio_data, int16_t* mfcc_out, MFCC_TypeDef *mfcc);
void MFCC_Destruct(MFCC_TypeDef *mfcc);

#endif //__MFCC_H_
