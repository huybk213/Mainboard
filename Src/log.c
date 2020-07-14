#include "log.h"
#include "stdio.h"
#include "main.h"
#include "stdarg.h"
#include "FreeRTOS.h"
#include "task.h"

#ifdef DEBUG_RTT
#include "SEGGER_RTT.h"
#endif
#include <stdio.h>
#include <ctype.h>
#include <string.h>

/* UART handler declared in "main.c" file */
//extern UART_HandleTypeDef UsbUartHandle;
extern void app_error_handler(char * file, int line, int error_code);

//PUTCHAR_PROTOTYPE
//{
//	//HAL_UART_Transmit(&UsbUartHandle, (uint8_t*)&ch, 1, 0xFFFF);
//	return ch;
//}

//void Rikkei_Logger_Init()
//{

//}

void rikkei_logger_hexdump(const char* tag, char* buffer, int buff_len)
{

}

void Rikkei_Logger_Fake_Func(){}


uint32_t rk_log_timestamp()
{
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) == 0)
    {
        return xTaskGetTickCount();
    }
    else
    {
        return xTaskGetTickCountFromISR();
    } 
}

void rk_log_write(rk_log_level_t level, const char* tag, const char* format, ...)
{
#ifdef DEBUG_RTT
	#define LOG_BUFFER_SIZE 256

	char buffer[LOG_BUFFER_SIZE];
	char * p = buffer;
//	p += snprintf(buffer, LOG_BUFFER_SIZE, "%s", tag);
	va_list args;
	va_start (args, format);
	vsnprintf(p, LOG_BUFFER_SIZE - (p - buffer), format, args);
	va_end (args);

	#undef LOG_BUFFER_SIZE

#ifdef DEBUG_RTT
	SEGGER_RTT_WriteString(0, buffer);
#endif

#endif /* DEBUG_RTT || DEBUG_UART */
}


void rk_log_buffer_hexdump_internal(const char *tag, const void *buffer, uint16_t buff_len, rk_log_level_t log_level)
{
	#define BYTES_PER_LINE 16
    char * t_buffer = (char*)buffer;
    if (buff_len == 0) {
        return;
    }
    char temp_buffer[BYTES_PER_LINE + 3]; //for not-byte-accessible memory
    const char *ptr_line;
    //format: field[length]
    // ADDR[10]+"   "+DATA_HEX[8*3]+" "+DATA_HEX[8*3]+"  |"+DATA_CHAR[8]+"|"
    char hd_buffer[10 + 3 + BYTES_PER_LINE * 3 + 3 + BYTES_PER_LINE + 1 + 1];
    char *ptr_hd;
    int bytes_cur_line;

    do {
        if (buff_len > BYTES_PER_LINE) {
            bytes_cur_line = BYTES_PER_LINE;
        } else {
            bytes_cur_line = buff_len;
        }
//        if (!esp_ptr_byte_accessible(buffer)) {
        	if (0) {
            //use memcpy to get around alignment issue
            memcpy(temp_buffer, buffer, (bytes_cur_line + 3) / 4 * 4);
            ptr_line = temp_buffer;
        } else {
            ptr_line = buffer;
        }
        ptr_hd = hd_buffer;

        ptr_hd += sprintf(ptr_hd, "%p ", buffer);
        for (int i = 0; i < BYTES_PER_LINE; i ++) {
            if ((i & 7) == 0) {
                ptr_hd += sprintf(ptr_hd, " ");
            }
            if (i < bytes_cur_line) {
                ptr_hd += sprintf(ptr_hd, " %02x", ptr_line[i]);
            } else {
                ptr_hd += sprintf(ptr_hd, "   ");
            }
        }
        ptr_hd += sprintf(ptr_hd, "  |");
        for (int i = 0; i < bytes_cur_line; i ++) {
            if (isprint((int)ptr_line[i])) {
                ptr_hd += sprintf(ptr_hd, "%c", ptr_line[i]);
            } else {
                ptr_hd += sprintf(ptr_hd, ".");
            }
        }
        ptr_hd += sprintf(ptr_hd, "|");

        RK_LOG_LEVEL(log_level, tag, "%s", hd_buffer);
        t_buffer += bytes_cur_line;
        buff_len -= bytes_cur_line;
    } while (buff_len);
}

