#ifndef __DNN_H__
#define __DNN_H__

#include "dnn_weights.h"
#include "arm_nnfunctions.h"
#include "arm_math.h"
#define NUM_MFCC_COEFFS 10

#define INPUT_DIM 250

#define DNN_DENSE_DIM 8
#define DNN_DENSE_WT_DIM (INPUT_DIM*DNN_DENSE_DIM)                  //(250,8)

#define DNN_OUTPUT_DIM 2                                        //(2,)
#define DNN_OUTPUT_WT_DIM (DNN_DENSE_DIM*DNN_OUTPUT_DIM)                //(8,2)

#define DNN_VECT_BUFF_SIZE 250
#define DNN_SCRATCH_BUFFER_SIZE (DNN_DENSE_DIM + DNN_VECT_BUFF_SIZE)

// class DNN_TypeDef 
// {
//   public:
//     DNN();
//     ~DNN();
    
//     void Run_NN(q15_t* in_data, q15_t* out_data);

//   private:
//     q15_t* scratch_pad;
//     q15_t* dense_out;        //(8,)
//     q15_t* vec_buffer;      //(250,)
    
//     // Hyper params    
//     static q7_t const dense_wt[DNN_DENSE_WT_DIM];
//     static q7_t const dense_bias[DNN_DENSE_DIM]; 
    
//     static q7_t const output_wt[DNN_OUTPUT_WT_DIM];
//     static q7_t const output_bias[DNN_OUTPUT_DIM];
// };
typedef struct{
    int16_t* scratch_pad;
    int16_t* dense_out;        //(8,)
    int16_t* vec_buffer;      //(250,) 
}DNN_TypeDef;

void DNN_Init(DNN_TypeDef *dnn);
void DNN_Destruct(DNN_TypeDef *dnn);
void Run_NN(int16_t* in_data, int16_t* out, DNN_TypeDef *dnn);

#endif
