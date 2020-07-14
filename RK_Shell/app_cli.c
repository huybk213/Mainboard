#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "main.h"
#include "log.h"

#include <app_cli.h>
#include <vsm_shell.h>
/* Logging and RTT */
#include "log.h"
//#include "rtt_input.h"
#ifdef DEBUG_RTT
#include "SEGGER_RTT.h"
#endif
#include "ringbuff/ringbuff.h"

#define TAG "cli"
#define RAW_BUFFER_LEN 32


static ringbuff_t m_ringbuffer_log;
static uint8_t m_raw_buffer[RAW_BUFFER_LEN+1];

static int32_t cli_reset_system                (p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_get_mac_ble                 (p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_send_audio                  (p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_record_audio                (p_shell_context_t context, int32_t argc, char **argv);

static const shell_command_context_t cli_command_table[] = {
    {"reset",       "\"reset\": reset system\r\n",                                          cli_reset_system,           0},
    {"getble",      "\tgetble : Get bluetooth mac address\r\n",                             cli_get_mac_ble,            0},
	{"send_audio",  "\tsend_audio X times: Send audio to esp32\r\n",                        cli_send_audio,            	1},
	{"record",      "\trecord : Record audio and send to cloud\r\n",                        cli_record_audio,           0},
};

static shell_context_struct user_context;


void user_put_char(uint8_t *buf, uint32_t len)
{
    SEGGER_RTT_Write(0, buf, len);
}

void cli_input_insert(uint8_t *buf, uint32_t len)
{
    ringbuff_write(&m_ringbuffer_log, buf, len);
}


int rtt_custom_printf(const char * format, ...)
{
    int r;
    va_list ParamList;
    extern int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList);
    va_start(ParamList, format);
    r = SEGGER_RTT_vprintf(0, format, &ParamList);
    va_end(ParamList);

    return r;
}

void user_get_char(uint8_t *buf, uint32_t len)
{
    uint8_t byte_read;
    if (ringbuff_read(&m_ringbuffer_log, &byte_read, 1))
    {
        *buf = byte_read;
    }
    else
    {
        *buf = 0xFF;
    }
}

void vsm_cli_start()
{
    ringbuff_init(&m_ringbuffer_log, m_raw_buffer, RAW_BUFFER_LEN+1);
    SHELL_SetContext(&user_context);
    SHELL_Init(&user_context,
            user_put_char,
            user_get_char,
            rtt_custom_printf,
            ">",
            false);

    /* Register CLI commands */
    for(int i = 0; i < sizeof(cli_command_table)/sizeof(shell_command_context_t); i ++) {
        SHELL_RegisterCommand(&cli_command_table[i]);
    }

    /* Run CLI task */
    SHELL_Main();
}

/* Reset System */
static int32_t cli_reset_system(p_shell_context_t context, int32_t argc, char **argv)
{
//    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "sys reset");
	RK_LOGI(TAG, "System reset\r\n");
	NVIC_SystemReset();
    return 0;
}


static int32_t cli_get_mac_ble(p_shell_context_t context, int32_t argc, char **argv)
{
//    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Get mac ble\n");
	RK_LOGI(TAG, "[%s] : Implement later\r\n", __FUNCTION__);
    return 0;
}

static int32_t cli_send_audio(p_shell_context_t context, int32_t argc, char **argv)
{
//	extern uint32_t send_first_time;
//	send_first_time = atoi(argv[1]);
//	RK_LOGI(TAG, "Send audio to ESP32 [%u] times\r\n", send_first_time);
	return 1;
}

static int32_t cli_record_audio                (p_shell_context_t context, int32_t argc, char **argv)
{
	return 1;
}
