#include "gru.h"
#include <stdlib.h>
#include "FreeRTOS.h"

extern void Assert_Is_NULL(void * arg);

// Hyper Params
const int8_t gru1_wt_z[GRU1_WT_DIM] = GRU1_WT_Z;                   //(10,64)
const int8_t gru1_wt_r[GRU1_WT_DIM] = GRU1_WT_R;                   //(10,64)
const int8_t gru1_wt_h[GRU1_WT_DIM] = GRU1_WT_H;                   //(10,64)
const int8_t gru1_bias_z[GRU1_NUM_TENSOR] = GRU1_BIAS_Z;           //(64)
const int8_t gru1_bias_r[GRU1_NUM_TENSOR] = GRU1_BIAS_R;           //(64)
const int8_t gru1_bias_h[GRU1_NUM_TENSOR] = GRU1_BIAS_H;           //(64)
const int8_t gru1_wt_rec_z[GRU1_WT_REC_DIM] = GRU1_WT_Rec_Z;           //(64,64)
const int8_t gru1_wt_rec_r[GRU1_WT_REC_DIM] = GRU1_WT_Rec_R;           //(64,64)
const int8_t gru1_wt_rec_h[GRU1_WT_REC_DIM] = GRU1_WT_Rec_H;           //(64,64)
const int8_t dense_wt_gru[DENSE_WT_DIM] = DENSE_WT;                    //(64,64)
const int8_t dense_bias_gru[DENSE_DIM] = DENSE_BIAS;                   //(64,)
const int8_t output_wt_gru[OUTPUT_WT_DIM] = OUTPUT_WT;                 //(64,7)
const int8_t output_bias_gru[OUTPUT_DIM] = OUTPUT_BIAS;                //(7,)

void Run_NN_GRU(int16_t* in_data, int16_t* out,GRU_TypeDef *gru)
{
//    int16_t* check1 = gru->temp_q15_buffer + 128;
//    int16_t* check2 = check1 + 64;
    
    memset(gru->rec_bias, 0, GRU1_NUM_TENSOR);
    memset(gru->gru1_hidden, 0, 128);
  
    for (int slide=0; slide < 25; slide++)
    {
        for (int i=0;i<10;i++) gru->input_buffer[i] = in_data[slide*10+i];
        Run_GRU(gru->input_buffer,
                gru->gru1_update,
                gru->gru1_reset,
                gru->gru1_hidden,
                gru->temp_q15_buffer,
                gru->vec_buffer,
                NUM_MFCC_COEFFS,
                GRU1_NUM_TENSOR,
                gru1_wt_z,
                gru1_wt_r,
                gru1_wt_h,
                gru1_bias_z,
                gru1_bias_r,
                gru1_bias_h,
                gru1_wt_rec_z,
                gru1_wt_rec_r,
                gru1_wt_rec_h,
                gru); 
    }
    
    // Relu (Dense (64,))
    // Q1.14 * Q0.7 * 64 => Q 8.21 -> 8.7
    arm_fully_connected_mat_q7_vec_q15(gru->gru1_hidden, dense_wt_gru, GRU1_NUM_TENSOR, DENSE_DIM, 14, 14, dense_bias_gru, gru->dense_out, gru->vec_buffer);
    arm_relu_q15(gru->dense_out, DENSE_DIM);
    
    // Softmax (Output (7,))
    // Q8.7 * Q0.7 * 2 => Q 10.14 + Q0.7 => Q10.5
    arm_fully_connected_mat_q7_vec_q15(gru->dense_out, output_wt_gru, DENSE_DIM, OUTPUT_DIM, 7, 8, output_bias_gru, out, gru->vec_buffer);
}

void GRU_Init(GRU_TypeDef *gru)
{
  memset(gru->rec_bias,0,GRU1_NUM_TENSOR);
  gru->scratch_pad       = pvPortMalloc(SCRATCH_BUFFER_SIZE * sizeof(int16_t));
  Assert_Is_NULL(gru->scratch_pad);
  gru->input_buffer      = gru->scratch_pad;                                //(10,): 00  -> 09
  gru->gru1_update       = gru->scratch_pad + NUM_MFCC_COEFFS;              //(64,): 10  -> 73
  gru->gru1_reset        = gru->gru1_update + GRU1_NUM_TENSOR;              //(64,): 74  -> 137
  gru->gru1_hidden       = gru->gru1_reset  + GRU1_NUM_TENSOR;              //(64,): 138 -> 201
  gru->dense_out         = gru->gru1_hidden + GRU1_NUM_TENSOR;              //(64,): 394 -> 457
  gru->temp_q15_buffer   = gru->dense_out + DENSE_DIM;
  gru->vec_buffer        = gru->temp_q15_buffer + TEMP_BUFFER_SIZE;       //(128,)  
}

void GRU_Destruct(GRU_TypeDef *gru)
{
  vPortFree(gru->scratch_pad);
}

void Run_GRU(  int16_t* in_data,
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
                    )
{
    int16_t* q15_buffer2 = q15_buffer + NUM_GRU_TENSOR;
 //   int16_t* check1 = q15_buffer2 + NUM_GRU_TENSOR;
//    int16_t* check2 = check1 + NUM_GRU_TENSOR;
    
    // ====================== UPDATE GATE ======================
    //rec_z  = hidden . Wrz             => temp_buffer2(64,) || Qx.y
    //Xz     = Input  . Wz + Bz;        => temp_buffer (64,) || Q5.3
    //UPDATE = hard_sigmoid(Xz + rec_z) => UPDATE      (64,) (hard_sigmoid -> sigmoid)
   
    

    // Q1.14 * Q0.7 * 64 -> Q 6.21 -> Q6.6
    arm_fully_connected_mat_q7_vec_q15(hidden, Wrz, NUM_GRU_TENSOR, NUM_GRU_TENSOR, 0, 15, gru->rec_bias, q15_buffer, vec_buffer);
    
    // Q5.2 * Q0.7 * 10 -> Q 8.9 -> Q8.6
    arm_fully_connected_mat_q7_vec_q15(in_data, Wz, INPUT_LEN, NUM_GRU_TENSOR, 2, 3, Bz, q15_buffer2, vec_buffer);
    
    arm_add_q15(q15_buffer, q15_buffer2, update, NUM_GRU_TENSOR); //Qx.6

    //HARD SIGMOID [-2,5;2,5] -> [0,1]: update * 0.2 + 0.5
    //Qx.6 * Q0.15 -> Qx.21 => Q1.14 (14 bit)
    arm_scale_q15(update,0x199a, 8, update, NUM_GRU_TENSOR); // auto shift 15 - outshift

    arm_offset_q15(update,0x2000,update,NUM_GRU_TENSOR); //+ 0.5 Q1.14

    for (int i = 0; i < NUM_GRU_TENSOR; i++)
    {
        if (update[i] < 0x0000) update[i] = 0; //Clip 0
        else if(update[i] > 0x4000) update[i] = 0x4000; //Clip 1
    }
    //=> UPDATE Q 1.14
    
    // ====================== RESET GATE ======================
    //rec_r = hidden . Wrr             => temp_buffer2(64,) || Qm.n * Q1.7
    //Xr    = Input  . Wr + Br;        => temp_buffer (64,) || Q5.3
    //RESET = hard_sigmoid(Xr + rec_r) => RESET       (64,)  (hard_sigmoid -> sigmoid)
    
    arm_fully_connected_mat_q7_vec_q15(hidden, Wrr, NUM_GRU_TENSOR, NUM_GRU_TENSOR, 0, 15, gru->rec_bias, q15_buffer, vec_buffer);
    arm_fully_connected_mat_q7_vec_q15(in_data, Wr, INPUT_LEN, NUM_GRU_TENSOR, 2, 3, Br, q15_buffer2, vec_buffer);
    arm_add_q15(q15_buffer2, q15_buffer, reset, NUM_GRU_TENSOR);
    arm_scale_q15(reset, 0x199a, 8, reset, NUM_GRU_TENSOR);
    arm_offset_q15(reset,0x2000, reset, NUM_GRU_TENSOR);
    for (int i = 0; i < NUM_GRU_TENSOR; i++)
    {
        if (reset[i] < 0) reset[i] = 0;
        else if(reset[i] > 0x4000) reset[i] = 0x4000;
    }
    //=> RESET Q 1.14

    // ====================== MAIN DATA ====================== Save HH => temp_buffer
    //recurrent_h = (reset* hidden) . Wrh => temp_buffer (64,)
    //Xh = Input . Wh + Bh;               => temp_buffer2(64,) Q5.3
    //hh = tanh (Xh + recurrent_h)        => temp_buffer (64,)

    //Q 1.14 * Q1.14  -> Q1.28 => Q1.13 => NONE
    arm_mult_q15(reset,hidden,reset,NUM_GRU_TENSOR); //Auto shift 15 bits
    
    //Q 1.13 * Q 0.7 * 64 => Q 6.20 => Q6.6
    arm_fully_connected_mat_q7_vec_q15(reset, Wrh, NUM_GRU_TENSOR, NUM_GRU_TENSOR, 0, 14, gru->rec_bias,  q15_buffer, vec_buffer);
    
    //Q5.2 * Q0.7 * 10 -> Q8.9 => Q8.6
    arm_fully_connected_mat_q7_vec_q15(in_data, Wh, INPUT_LEN, NUM_GRU_TENSOR, 2, 3, Bh, q15_buffer2, vec_buffer);
    
    //Q x.6 (DATA)
    arm_add_q15(q15_buffer,q15_buffer2,q15_buffer,NUM_GRU_TENSOR);
    
    //Qx.6=> Q3.12
    arm_shift_q15(q15_buffer,6,q15_buffer,NUM_GRU_TENSOR);
    
    
    arm_nn_activations_direct_q15(q15_buffer, NUM_GRU_TENSOR, 3, ARM_TANH);//Q0.15 ===> DONE

    // ====================== UPDATE HIDDEN ======================   //No Quantize needed
    //UPDATE * HIDDEN
    
    //Q1.14 * Q1.14  => Q1.28 => Q1.13 (28 bit)
    arm_mult_q15(update, hidden, hidden, NUM_GRU_TENSOR);
    
    //(update - 1)*hh
    arm_offset_q15(update, 0xc000, update, NUM_GRU_TENSOR);
    
    //Q1.14 * Q0.15 => Q1.14
    arm_mult_q15  (update, q15_buffer, update, NUM_GRU_TENSOR);
    
    //hidden = update* hidden - (update-1) * hh
    //Q1.13 + Q 1.14 -> Q2.13
    arm_shift_q15(update,-1,update,NUM_GRU_TENSOR);
    arm_sub_q15(hidden, update, hidden, NUM_GRU_TENSOR); //=> Q 2.13
    arm_shift_q15(hidden,1,hidden,NUM_GRU_TENSOR); // =>Q1.14
}
/*
//Input buffer(10,) ->  GRU1(64*3,) -> GRU2(64*3,) -> DENSE(64,) -> OUTPUT(7,)
//             (25,10)|        (25,64)|         (64,)|        (64,)|


 
void GRU::Run_GRU(  q15_t* in_data,
                    q15_t* update,
                    q15_t* reset,
                    q15_t* hidden,
                    q15_t* q15_buffer,
                    q15_t* vec_buffer,
                    const uint16_t INPUT_LEN,
                    const uint16_t NUM_GRU_TENSOR,
                    const q7_t*  Wz,
                    const q7_t*  Wr,
                    const q7_t*  Wh,
                    const q7_t*  Bz,
                    const q7_t*  Br,
                    const q7_t*  Bh,
                    const q7_t*  Wrz,
                    const q7_t*  Wrr,
                    const q7_t*  Wrh
                    )
{
    q15_t* q15_buffer2 = q15_buffer + NUM_GRU_TENSOR;
    q15_t* check1 = q15_buffer2 + NUM_GRU_TENSOR;
    q15_t* check2 = check1 + NUM_GRU_TENSOR;
    
    // ====================== UPDATE GATE ======================
    //rec_z  = hidden . Wrz             => temp_buffer2(64,) || Qx.y
    //Xz     = Input  . Wz + Bz;        => temp_buffer (64,) || Q5.3
    //UPDATE = hard_sigmoid(Xz + rec_z) => UPDATE      (64,) (hard_sigmoid -> sigmoid)
   
    

    // Q1.14 * Q0.7 * 64 -> Q 6.21 -> Q6.6
    arm_fully_connected_mat_q7_vec_q15(hidden, Wrz, NUM_GRU_TENSOR, NUM_GRU_TENSOR, 0, 15, rec_bias, q15_buffer, vec_buffer);
    
    // Q5.2 * Q0.7 * 10 -> Q 8.9 -> Q8.6
    arm_fully_connected_mat_q7_vec_q15(in_data, Wz, INPUT_LEN, NUM_GRU_TENSOR, 2, 3, Bz, q15_buffer2, vec_buffer);
    
    arm_add_q15(q15_buffer, q15_buffer2, update, NUM_GRU_TENSOR); //Qx.6

    //HARD SIGMOID [-2,5;2,5] -> [0,1]: update * 0.2 + 0.5
    //Qx.6 * Q0.15 -> Qx.21 => Q1.14 (14 bit)
    arm_scale_q15(update,0x199a, 8, update, NUM_GRU_TENSOR); // auto shift 15 - outshift

    arm_offset_q15(update,0x2000,update,NUM_GRU_TENSOR); //+ 0.5 Q1.14

    for (int i = 0; i < NUM_GRU_TENSOR; i++)
    {
        if (update[i] < 0x0000) update[i] = 0; //Clip 0
        else if(update[i] > 0x4000) update[i] = 0x4000; //Clip 1
    }
    //=> UPDATE Q 1.14
    
    // ====================== RESET GATE ======================
    //rec_r = hidden . Wrr             => temp_buffer2(64,) || Qm.n * Q1.7
    //Xr    = Input  . Wr + Br;        => temp_buffer (64,) || Q5.3
    //RESET = hard_sigmoid(Xr + rec_r) => RESET       (64,)  (hard_sigmoid -> sigmoid)
    
    arm_fully_connected_mat_q7_vec_q15(hidden, Wrr, NUM_GRU_TENSOR, NUM_GRU_TENSOR, 0, 15, rec_bias, q15_buffer, vec_buffer);
    arm_fully_connected_mat_q7_vec_q15(in_data, Wr, INPUT_LEN, NUM_GRU_TENSOR, 2, 3, Br, q15_buffer2, vec_buffer);
    arm_add_q15(q15_buffer2, q15_buffer, reset, NUM_GRU_TENSOR);
    arm_scale_q15(reset, 0x199a, 8, reset, NUM_GRU_TENSOR);
    arm_offset_q15(reset,0x2000, reset, NUM_GRU_TENSOR);
    for (int i = 0; i < NUM_GRU_TENSOR; i++)
    {
        if (reset[i] < 0) reset[i] = 0;
        else if(reset[i] > 0x4000) reset[i] = 0x4000;
    }
    //=> RESET Q 1.14

    // ====================== MAIN DATA ====================== Save HH => temp_buffer
    //recurrent_h = (reset* hidden) . Wrh => temp_buffer (64,)
    //Xh = Input . Wh + Bh;               => temp_buffer2(64,) Q5.3
    //hh = tanh (Xh + recurrent_h)        => temp_buffer (64,)

    //Q 1.14 * Q1.14  -> Q1.28 => Q1.13 => NONE
    arm_mult_q15(reset,hidden,reset,NUM_GRU_TENSOR); //Auto shift 15 bits
    
    //Q 1.13 * Q 0.7 * 64 => Q 6.20 => Q6.6
    arm_fully_connected_mat_q7_vec_q15(reset, Wrh, NUM_GRU_TENSOR, NUM_GRU_TENSOR, 0, 14, rec_bias,  q15_buffer, vec_buffer);
    
    //Q5.2 * Q0.7 * 10 -> Q8.9 => Q8.6
    arm_fully_connected_mat_q7_vec_q15(in_data, Wh, INPUT_LEN, NUM_GRU_TENSOR, 2, 3, Bh, q15_buffer2, vec_buffer);
    
    //Q x.6 (DATA)
    arm_add_q15(q15_buffer,q15_buffer2,q15_buffer,NUM_GRU_TENSOR);
    
    //Qx.6=> Q3.12
    arm_shift_q15(q15_buffer,6,q15_buffer,NUM_GRU_TENSOR);
    
    
    arm_nn_activations_direct_q15(q15_buffer, NUM_GRU_TENSOR, 3, ARM_TANH);//Q0.15 ===> DONE

    // ====================== UPDATE HIDDEN ======================   //No Quantize needed
    //UPDATE * HIDDEN
    
    //Q1.14 * Q1.14  => Q1.28 => Q1.13 (28 bit)
    arm_mult_q15(update, hidden, hidden, NUM_GRU_TENSOR);
    
    //(update - 1)*hh
    arm_offset_q15(update, 0xc000, update, NUM_GRU_TENSOR);
    
    //Q1.14 * Q0.15 => Q1.14
    arm_mult_q15  (update, q15_buffer, update, NUM_GRU_TENSOR);
    
    //hidden = update* hidden - (update-1) * hh
    //Q1.13 + Q 1.14 -> Q2.13
    arm_shift_q15(update,-1,update,NUM_GRU_TENSOR);
    arm_sub_q15(hidden, update, hidden, NUM_GRU_TENSOR); //=> Q 2.13
    arm_shift_q15(hidden,1,hidden,NUM_GRU_TENSOR); // =>Q1.14
}

void GRU::Run_NN(q15_t* in_data, q15_t* out)
{
    q15_t* check1 = temp_q15_buffer + 128;
    q15_t* check2 = check1 + 64;

    memset(gru1_hidden,0,128);
  
    for (int slide=0;slide < 25;slide++)
    {
        for (int i=0;i<10;i++) input_buffer[i] = in_data[slide*10+i];
        Run_GRU(input_buffer,
                gru1_update,
                gru1_reset,
                gru1_hidden,
                temp_q15_buffer,
                vec_buffer,
                NUM_MFCC_COEFFS,
                GRU1_NUM_TENSOR,
                gru1_wt_z,
                gru1_wt_r,
                gru1_wt_h,
                gru1_bias_z,
                gru1_bias_r,
                gru1_bias_h,
                gru1_wt_rec_z,
                gru1_wt_rec_r,
                gru1_wt_rec_h); 
    }
    
    // Relu (Dense (64,))
    // Q1.14 * Q0.7 * 64 => Q 8.21 -> 8.7
    arm_fully_connected_mat_q7_vec_q15(gru1_hidden, dense_wt, GRU1_NUM_TENSOR, DENSE_DIM, 14, 14, dense_bias, dense_out, vec_buffer);
    arm_relu_q15(dense_out, DENSE_DIM);
    
    // Softmax (Output (7,))
    // Q8.7 * Q1.6 * 8 => Q 13.13 + Q0.7 => Q13.2
    arm_fully_connected_mat_q7_vec_q15(dense_out, output_wt, DENSE_DIM, OUTPUT_DIM, 6, 11, output_bias, out, vec_buffer);
}
*/
