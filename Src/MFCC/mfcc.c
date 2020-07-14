#include "mfcc.h"
#include "FreeRTOS.h"

float ** create_mel_fbank(MFCC_TypeDef *mfcc);
static inline float MelScale(float freq);
//static inline float InverseMelScale(float mel_freq);
float * create_dct_matrix(int32_t input_length, int32_t coefficient_count);
extern void Assert_Is_NULL(void* value);

/*****************************************************************************
******************************************************************************/
void MFCC_Init(MFCC_TypeDef *mfcc,int Num_MFCC_Features, int Frame_Len, int MFCC_Dec_Bits){
    // Round-up to nearest power of 2.
    mfcc->num_mfcc_features = Num_MFCC_Features;
    mfcc->frame_len = Frame_Len;
    mfcc->mfcc_dec_bits = MFCC_Dec_Bits;

    mfcc->frame_len_padded = 1024;//pow(2,ceil(log(mfcc->frame_len)/log(2)));
    mfcc->frame = pvPortMalloc(mfcc->frame_len_padded * sizeof(mfcc->frame));
    Assert_Is_NULL(mfcc->frame);

    mfcc->buffer = pvPortMalloc(mfcc->frame_len_padded * sizeof(mfcc->buffer));
    Assert_Is_NULL(mfcc->buffer);

    mfcc->mel_energies = pvPortMalloc(NUM_FBANK_BINS * sizeof(mfcc->mel_energies));
    Assert_Is_NULL(mfcc->mel_energies);
    //create window function
    mfcc->window_func = pvPortMalloc(mfcc->frame_len * sizeof(mfcc->window_func));
    Assert_Is_NULL(mfcc->window_func);
    
    for (int i = 0; i < mfcc->frame_len; i++)
        mfcc->window_func[i] = 0.5f - 0.5f*cos(M_2PI * ((float)i) / (mfcc->frame_len));
    
    //create mel filterbank
    mfcc->fbank_filter_first = pvPortMalloc(NUM_FBANK_BINS * sizeof(mfcc->fbank_filter_first));
    Assert_Is_NULL(mfcc->fbank_filter_first);
    
    mfcc->fbank_filter_last = pvPortMalloc(NUM_FBANK_BINS * sizeof(mfcc->fbank_filter_last));
    Assert_Is_NULL(mfcc->fbank_filter_last);

    mfcc->mel_fbank = create_mel_fbank(mfcc);
    
    //create DCT matrix
    mfcc->dct_matrix = create_dct_matrix(NUM_FBANK_BINS, mfcc->num_mfcc_features);
    
    //initialize FFT
    mfcc->rfft = pvPortMalloc(1 * sizeof(arm_rfft_fast_instance_f32));
    Assert_Is_NULL(mfcc->rfft);

    arm_rfft_fast_init_f32(mfcc->rfft, mfcc->frame_len_padded);
}

/*****************************************************************************
******************************************************************************/
static volatile uint32_t repeate = 0;
void MFCC_Compute(const float * audio_data, int16_t* mfcc_out, MFCC_TypeDef *mfcc) 
{
    repeate++;
  ////////////////////////////
  int32_t i, j, bin;
  for (i = 0; i < mfcc->frame_len; i++) 
  {
    mfcc->frame[i] = (float)audio_data[i];      // hardfault
  }
  
  //Fill up remaining with zeros
  memset(&mfcc->frame[mfcc->frame_len], 0, sizeof(float) * (mfcc->frame_len_padded-mfcc->frame_len));

  for (i = 0; i < mfcc->frame_len; i++)
  {
    mfcc->frame[i] *= mfcc->window_func[i];
  }
   
  if (repeate)
  {
       j = 0;
  }
  //Compute FFT
  arm_rfft_fast_f32(mfcc->rfft, mfcc->frame, mfcc->buffer, 0);

  //Convert to power spectrum
  //frame is stored as [real0, realN/2-1, real1, im1, real2, im2, ...]
  int32_t half_dim = mfcc->frame_len_padded/2;
  float first_energy = mfcc->buffer[0] * mfcc->buffer[0],
        last_energy =  mfcc->buffer[1] * mfcc->buffer[1];  // handle this special case
  for (i = 1; i < half_dim; i++)
  {
    float real = mfcc->buffer[i*2], im = mfcc->buffer[i*2 + 1];
    mfcc->buffer[i] = real*real + im*im;
  }
  mfcc->buffer[0] = first_energy;
  mfcc->buffer[half_dim] = last_energy;  
 
  float sqrt_data;
  //Apply mel filterbanks
  for (bin = 0; bin < NUM_FBANK_BINS; bin++) 
  {
    j = 0;
    float mel_energy = 0;
    int32_t first_index = mfcc->fbank_filter_first[bin];
    int32_t last_index = mfcc->fbank_filter_last[bin];
    for (i = first_index; i <= last_index; i++) 
    {
      arm_sqrt_f32(mfcc->buffer[i],&sqrt_data);
      mel_energy += (sqrt_data) * mfcc->mel_fbank[bin][j++];
    }
    mfcc->mel_energies[bin] = mel_energy;

    //avoid log of zero
    if (mel_energy == 0.0f)
      mfcc->mel_energies[bin] = FLT_MIN;
  }

  //Take log
  for (bin = 0; bin < NUM_FBANK_BINS; bin++)
    mfcc->mel_energies[bin] = logf(mfcc->mel_energies[bin]);

  //Take DCT. Uses matrix mul.
  for (i = 0; i < mfcc->num_mfcc_features; i++) 
  {
    float sum = 0.0f;
    for (j = 0; j < NUM_FBANK_BINS; j++) 
    {
      sum += mfcc->dct_matrix[i*NUM_FBANK_BINS+j] * mfcc->mel_energies[j];
    }

    //Input is Qx.mfcc_dec_bits (from quantization step)
    sum *= (0x1<<mfcc->mfcc_dec_bits);
    sum = round(sum); 
    if(sum >= 127)
      mfcc_out[i] = 127;
    else if(sum <= -128)
      mfcc_out[i] = -128;
    else
      mfcc_out[i] = sum; 
  }
}

/*****************************************************************************
******************************************************************************/

float * create_dct_matrix(int32_t input_length, int32_t coefficient_count){
  int32_t k, n;
  float * M = pvPortMalloc(input_length*coefficient_count * sizeof(float));

  float normalizer;
  arm_sqrt_f32(2.0f/(float)input_length,&normalizer);
  for (k = 0; k < coefficient_count; k++) {
    for (n = 0; n < input_length; n++) {
      M[k*input_length+n] = normalizer * cos( ((double)3.14)/input_length * (n + 0.5f) * k );
    } //minhda: dangerous used
  }
  return M;    
}
/*****************************************************************************
******************************************************************************/
float ** create_mel_fbank(MFCC_TypeDef *mfcc){
  
  int32_t bin, i;

  int32_t num_fft_bins = mfcc->frame_len_padded/2;
  float fft_bin_width = ((float)SAMP_FREQ) / mfcc->frame_len_padded;
  float mel_low_freq = MelScale(MEL_LOW_FREQ);
  float mel_high_freq = MelScale(MEL_HIGH_FREQ); 
  float mel_freq_delta = (mel_high_freq - mel_low_freq) / (NUM_FBANK_BINS+1);

  float *this_bin = pvPortMalloc(num_fft_bins * sizeof(float));
    
  float ** mel_fbank =  pvPortMalloc(NUM_FBANK_BINS * sizeof(float));

  for (bin = 0; bin < NUM_FBANK_BINS; bin++) {

    float left_mel = mel_low_freq + bin * mel_freq_delta;
    float center_mel = mel_low_freq + (bin + 1) * mel_freq_delta;
    float right_mel = mel_low_freq + (bin + 2) * mel_freq_delta;

    int32_t first_index = -1, last_index = -1;

    for (i = 0; i < num_fft_bins; i++) {

      float freq = (fft_bin_width * i);  // center freq of this fft bin.
      float mel = MelScale(freq);
      this_bin[i] = 0.0f;

      if (mel > left_mel && mel < right_mel) {
        float weight;
        if (mel <= center_mel) {
          weight = (mel - left_mel) / (center_mel - left_mel);
        } else {
          weight = (right_mel-mel) / (right_mel-center_mel);
        }
        this_bin[i] = weight;
        if (first_index == -1)
          first_index = i;
        last_index = i;
      }
    }

    mfcc->fbank_filter_first[bin] = first_index;
    mfcc->fbank_filter_last[bin] = last_index;
    mel_fbank[bin] = pvPortMalloc((last_index-first_index+1) * sizeof(float)); 

    int32_t j = 0;
    //copy the part we care about
    for (i = first_index; i <= last_index; i++) {
      mel_fbank[bin][j++] = this_bin[i];
    }
  }
  vPortFree(this_bin);
  return mel_fbank;
}
/*****************************************************************************
******************************************************************************/
static inline float MelScale(float freq) {
    return 1127.0f * logf (1.0f + freq / 700.0f);
}
/*****************************************************************************
******************************************************************************/
//static inline float InverseMelScale(float mel_freq) {
//  return 700.0f * (expf (mel_freq / 1127.0f) - 1.0f);
//}
/*****************************************************************************
******************************************************************************/
void MFCC_Destruct(MFCC_TypeDef *mfcc){
    vPortFree(mfcc->frame);
    vPortFree(mfcc->buffer);
    vPortFree(mfcc->mel_energies);
    vPortFree(mfcc->window_func);
    vPortFree(mfcc->fbank_filter_first);
    vPortFree(mfcc->fbank_filter_last);
    vPortFree(mfcc->dct_matrix);
    vPortFree(mfcc->rfft);
    for(int i=0;i<NUM_FBANK_BINS;i++){
        vPortFree(mfcc->mel_fbank[i]);
    }
    vPortFree(mfcc->mel_fbank);
}
