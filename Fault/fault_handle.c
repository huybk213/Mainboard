#include "fault_handle.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "log.h"

#define TAG "fault"

//void dump_error_msg(const char * err_msg);
static void dump_usage_error_msg(uint32_t CFSR_value);
static void dump_bus_fault_error_msg(uint32_t CFSR_value);
static void dump_memory_management_error_msg(uint32_t CFSR_value);
static void stack_dump(uint32_t stack[]);

void hard_fault_handler(uint32_t stack[])
{
//  static char msg[80];
  //if((CoreDebug->DHCSR & 0x01) != 0) {
    
  RK_LOGE(TAG, "Hard fault handler\r\nSCB->HFSR = 0x%08x\r\n\r\n", SCB->HFSR);
  if ((SCB->HFSR & (1 << 30)) != 0) 
  {
    RK_LOGE(TAG, "Forced Hard Fault\r\nSCB->CFSR = 0x%08x\r\n", SCB->CFSR);
    if((SCB->CFSR & 0xFFFF0000) != 0) 
    {
      dump_usage_error_msg(SCB->CFSR);
    } 
    if((SCB->CFSR & 0xFF00) != 0) 
    {
      dump_bus_fault_error_msg(SCB->CFSR);
    }
    if((SCB->CFSR & 0xFF) != 0) 
    {
      dump_memory_management_error_msg(SCB->CFSR);
    }      
  }  
  stack_dump(stack);
  __ASM volatile("BKPT #01");   // insert breakpoint here
  //}
   while(1);
}

//void dump_error_msg(const char * err_msg)
//{
//   while(*err_msg != '\0')
//	 {
//      ITM_SendChar(*err_msg);
//      ++err_msg;
//   }
//}

static void dump_usage_error_msg(uint32_t CFSR_value)
{
   RK_LOGE(TAG, "Usage fault: ");
   CFSR_value >>= 16; // right shift to lsb
   if((CFSR_value & (1<<9)) != 0) 
   {
      RK_LOGE(TAG, "Divide by zero\r\n");
   }
}

void dump_bus_fault_error_msg(uint32_t CFSR_value)
{
   RK_LOGE(TAG, "Bus fault: ");
   CFSR_value = ((CFSR_value & 0x0000FF00) >> 8); // mask and right shift to lsb
}

static void dump_memory_management_error_msg(uint32_t CFSR_value)
{
   RK_LOGE(TAG, "Memory Management fault: ");
   CFSR_value &= 0x000000FF; // mask just mem faults
}

#if defined(__CC_ARM)
__asm void HardFault_Handler(void)
{
   TST lr, #4
   ITE EQ
   MRSEQ r0, MSP
   MRSNE r0, PSP
   B __cpp(hard_fault_handler)
}
#elif defined(__ICCARM__)
void HardFault_Handler(void)
{
   __asm(   "TST lr, #4          \n"
            "ITE EQ              \n"
            "MRSEQ r0, MSP       \n"
            "MRSNE r0, PSP       \n"
            "B HardFaultHandler\n"
   );
}
#else
  #warning Not supported compiler type
#endif

enum { r0, r1, r2, r3, r12, lr, pc, psr };

static void stack_dump(uint32_t stack[])
{
  RK_LOGE(TAG, "r0  = 0x%08x\n", stack[r0]);
  RK_LOGE(TAG, "r1  = 0x%08x\n", stack[r1]);
  RK_LOGE(TAG, "r2  = 0x%08x\n", stack[r2]);
  RK_LOGE(TAG, "r3  = 0x%08x\n", stack[r3]);
  RK_LOGE(TAG, "r12 = 0x%08x\n", stack[r12]);
  RK_LOGE(TAG, "lr  = 0x%08x\n", stack[lr]);
  uint32_t fault_addr = stack[pc];
  RK_LOGE(TAG, "Please check function at : pc  = 0x%08x\n", fault_addr);    /* Here !!!!!!*/
  RK_LOGE(TAG, "psr = 0x%08x\n", stack[psr]);
}
