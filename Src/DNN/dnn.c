#include "dnn.h"
#include <stdlib.h>
#include "FreeRTOS.h"

extern void Assert_Is_NULL(void* value);

const int8_t dense_wt[DNN_DENSE_WT_DIM] = DNN_DENSE_WT;
const int8_t dense_bias[DNN_DENSE_DIM] = DNN_DENSE_BIAS;
const int8_t output_wt[DNN_OUTPUT_WT_DIM] = DNN_OUT_WT; 
const int8_t output_bias[DNN_OUTPUT_DIM] = DNN_OUT_BIAS;

void DNN_Init(DNN_TypeDef *dnn){
    dnn->scratch_pad       = (int16_t*)pvPortMalloc(DNN_SCRATCH_BUFFER_SIZE*sizeof(int16_t));
		Assert_Is_NULL(dnn->scratch_pad);
    dnn->dense_out         = dnn->scratch_pad;
    dnn->vec_buffer        = dnn->dense_out + DNN_DENSE_DIM; 
}

void DNN_Destruct(DNN_TypeDef *dnn){
    vPortFree(dnn->scratch_pad);
}

void Run_NN(q15_t* in_data, q15_t* out, DNN_TypeDef *dnn){
    // Relu (Dense (8,))
    // Q5.2 * Q1.6 * 8 => Q 9.8 + 0.7
    arm_fully_connected_mat_q7_vec_q15(in_data, dense_wt, INPUT_DIM, DNN_DENSE_DIM, 1, 2, dense_bias, dnn->dense_out, dnn->vec_buffer);
    arm_relu_q15(dnn->dense_out, DNN_DENSE_DIM);
    // Softmax (Output (2,))
    // Q9.8 * Q1.6 * 2 => Q 11.14 + Q2.5 => Q11.14 => Q11.5
    arm_fully_connected_mat_q7_vec_q15(dnn->dense_out, output_wt, DNN_DENSE_DIM, DNN_OUTPUT_DIM, 9, 10, output_bias, out, dnn->vec_buffer);
}

