#ifndef __GRU_H__
#define __GRU_H__



#include "gru_weights.h"
#include "arm_nnfunctions.h"
#include "arm_math.h"
#define NUM_MFCC_COEFFS 10
#define GRU1_NUM_TENSOR 64                                  //(64,)
#define GRU1_WT_DIM (NUM_MFCC_COEFFS*GRU1_NUM_TENSOR)       //(10,64)
#define GRU1_WT_REC_DIM (GRU1_NUM_TENSOR*GRU1_NUM_TENSOR)   //(64,64)

#define DENSE_DIM 64                                        //(64,)
#define DENSE_WT_DIM (GRU1_NUM_TENSOR*DENSE_DIM)            //(64,64)

// #define OUTPUT_DIM 8                                        //(7,)  // original
#define OUTPUT_DIM 2                                        //(7,)
#define OUTPUT_WT_DIM (DENSE_DIM*OUTPUT_DIM)                //(64,7)

#define VECT_BUFF_SIZE 128                                  //(128)
#define TEMP_BUFFER_SIZE 64*4

#define SCRATCH_BUFFER_SIZE (NUM_MFCC_COEFFS + 3*GRU1_NUM_TENSOR + DENSE_DIM + TEMP_BUFFER_SIZE + VECT_BUFF_SIZE)
// class GRU 
// {
//   public:
//     GRU();
//     ~GRU();
    
//     void Run_GRU(   q15_t* in_data,
//                     q15_t* update,
//                     q15_t* reset,
//                     q15_t* hidden,
//                     q15_t* q15_buffer,
//                     q15_t* vec_buffer,
//                     const uint16_t INPUT_LEN,
//                     const uint16_t NUM_GRU_TENSOR,
//                     const q7_t*  Wz,
//                     const q7_t*  Wr,
//                     const q7_t*  Wh,
//                     const q7_t*  Bz,
//                     const q7_t*  Br,
//                     const q7_t*  Bh,
//                     const q7_t*  Wrz,
//                     const q7_t*  Wrr,
//                     const q7_t*  Wrh
//                     );
                    
//     void Run_NN(q15_t* in_data, q15_t* out_data);

//   private:
//     q7_t rec_bias [GRU1_NUM_TENSOR];
//     // data buffer
//     q15_t* scratch_pad;
//     q15_t* input_buffer;     //(10,)
    
//     q15_t* gru1_update;      //(64,)
//     q15_t* gru1_reset;       //(64,)
//     q15_t* gru1_hidden;      //(64,)
    
//     q15_t* dense_out;        //(64,)
    
//     q15_t* temp_q15_buffer; //(128,)
//     q15_t* vec_buffer;      //(128,)
    
//     // Hyper params
//     static q7_t const gru1_wt_z[GRU1_WT_DIM];                      //(10,64)
//     static q7_t const gru1_wt_r[GRU1_WT_DIM];                      //(10,64)
//     static q7_t const gru1_wt_h[GRU1_WT_DIM];                      //(10,64)
//     static q7_t const gru1_bias_z[GRU1_NUM_TENSOR];                //(64)
//     static q7_t const gru1_bias_r[GRU1_NUM_TENSOR];                //(64)
//     static q7_t const gru1_bias_h[GRU1_NUM_TENSOR];                //(64)
//     static q7_t const gru1_wt_rec_z[GRU1_WT_REC_DIM];              //(64,64)
//     static q7_t const gru1_wt_rec_r[GRU1_WT_REC_DIM];              //(64,64)
//     static q7_t const gru1_wt_rec_h[GRU1_WT_REC_DIM];              //(64,64)
    
//     static q7_t const dense_wt[DENSE_WT_DIM];                      //(64,64)
//     static q7_t const dense_bias[DENSE_DIM];                       //(64,)
    
//     static q7_t const output_wt[OUTPUT_WT_DIM];                    //(64,7)
//     static q7_t const output_bias[OUTPUT_DIM];                     //(7,)
// };

typedef struct {
    int8_t rec_bias [GRU1_NUM_TENSOR];
    // data buffer
    int16_t* scratch_pad;
    int16_t* input_buffer;     //(10,)
    int16_t* gru1_update;      //(64,)
    int16_t* gru1_reset;       //(64,)
    int16_t* gru1_hidden;      //(64,)
    int16_t* dense_out;        //(64,)
    int16_t* temp_q15_buffer; //(128,)
    int16_t* vec_buffer;      //(128,)
}GRU_TypeDef;

void Run_GRU(   int16_t* in_data,
                int16_t* update,
                int16_t* reset,
                int16_t* hidden,
                int16_t* q15_buffer,
                int16_t* vec_buffer,
                const uint16_t INPUT_LEN,
                const uint16_t NUM_GRU_TENSOR,
                const int8_t*  Wz,
                const int8_t*  Wr,
                const int8_t*  Wh,
                const int8_t*  Bz,
                const int8_t*  Br,
                const int8_t*  Bh,
                const int8_t*  Wrz,
                const int8_t*  Wrr,
                const int8_t*  Wrh,
                GRU_TypeDef *gru
                );

void Run_NN_GRU(int16_t* in_data, int16_t* out, GRU_TypeDef *gru);
void GRU_Init(GRU_TypeDef *gru);
void GRU_Destruct(GRU_TypeDef *gru);

#endif
