#include <assert.h>
#include "vsm_shell.h"
#include "string.h"


#define KEY_ESC (0x1BU)
#define KET_DEL (0x7FU)


static int32_t HelpCommand(p_shell_context_t context, int32_t argc, char **argv); /*!< help command */

static int32_t ExitCommand(p_shell_context_t context, int32_t argc, char **argv); /*!< exit command */

static int32_t ParseLine(const char *cmd, uint32_t len, char *argv[SHELL_MAX_ARGS]); /*!< parse line command */

static int32_t StrCompare(const char *str1, const char *str2, int32_t count); /*!< compare string command */

static void ProcessCommand(p_shell_context_t context, const char *cmd); /*!< process a command */

static void GetHistoryCommand(p_shell_context_t context, uint8_t hist_pos); /*!< get commands history */

static void AutoComplete(p_shell_context_t context); /*!< auto complete command */

static uint8_t GetChar(p_shell_context_t context); /*!< get a char from communication interface */

static int32_t StrLen(const char *str); /*!< get string length */

static char *StrCopy(char *dest, const char *src, int32_t count); /*!< string copy */

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const shell_command_context_t xHelpCommand = {"help", "\r\n\"help\": Lists all the registered commands\r\n",
                                                     HelpCommand, 0};

static const shell_command_context_t xExitCommand = {"exit", "\r\n\"exit\": Exit program\r\n", ExitCommand, 0};

static shell_command_context_list_t g_RegisteredCommands;

static char g_paramBuffer[SHELL_BUFFER_SIZE];

/*******************************************************************************
 * Code
 ******************************************************************************/
static bool m_loop_back = false;
void SHELL_Init(
    p_shell_context_t context, send_data_cb_t send_cb, recv_data_cb_t recv_cb, printf_data_t shell_printf, char *prompt, bool loop_back)
{
//    assert(send_cb != NULL);
//    assert(recv_cb != NULL);
//    assert(prompt != NULL);
//    assert(shell_printf != NULL);

    /* Memset for context */
    memset(context, 0, sizeof(shell_context_struct));
    context->send_data_func = send_cb;
    context->recv_data_func = recv_cb;
    context->printf_data_func = shell_printf;
    context->prompt = prompt;
    
    m_loop_back = loop_back;
    SHELL_RegisterCommand(&xHelpCommand);
    SHELL_RegisterCommand(&xExitCommand);
}

static bool printed = false;
static p_shell_context_t m_context; 
void SHELL_SetContext(p_shell_context_t context)
{
    m_context = context;
}

int32_t SHELL_Main()
{
    uint8_t ch;
    int32_t i;

    if (!m_context)
    {
        return -1;
    }
    if (printed == false)
    {
        m_context->exit = false;
        m_context->printf_data_func("\r\nSHELL (build: %s)\r\n", __DATE__);
        m_context->printf_data_func("Copyright (c) RikkeiSoft JSC\r\n");
        m_context->printf_data_func(m_context->prompt);
        printed = true;
    }

    // while (1)
    {
        if (m_context->exit)
        {
            return -1;
        }
        ch = GetChar(m_context);
        
        if (ch == 0xFF) //invalid input
          return 0;

        /* Special key */
        if (ch == KEY_ESC)
        {
            m_context->stat = kSHELL_Special;
            return 0;
        }
        else if (m_context->stat == kSHELL_Special)
        {
            /* Function key */
            if (ch == '[')
            {
                m_context->stat = kSHELL_Function;
                return 0;
            }
            m_context->stat = kSHELL_Normal;
        }
        else if (m_context->stat == kSHELL_Function)
        {
            m_context->stat = kSHELL_Normal;

            switch ((uint8_t)ch)
            {
                /* History operation here */
                case 'A': /* Up key */
                    GetHistoryCommand(m_context, m_context->hist_current);
                    if (m_context->hist_current < (m_context->hist_count - 1))
                    {
                        m_context->hist_current++;
                    }
                    break;
                case 'B': /* Down key */
                    GetHistoryCommand(m_context, m_context->hist_current);
                    if (m_context->hist_current > 0)
                    {
                        m_context->hist_current--;
                    }
                    break;
                case 'D': /* Left key */
                    if (m_context->c_pos)
                    {
                        m_context->printf_data_func("\b");
                        m_context->c_pos--;
                    }
                    break;
                case 'C': /* Right key */
                    if (m_context->c_pos < m_context->l_pos)
                    {
                        m_context->printf_data_func("%c", m_context->line[m_context->c_pos]);
                        m_context->c_pos++;
                    }
                    break;
                default:
                    break;
            }
            return 0;
        }
        /* Handle tab key */
        else if (ch == '\t')
        {
#if SHELL_AUTO_COMPLETE
            /* Move the cursor to the beginning of line */
            for (i = 0; i < m_context->c_pos; i++)
            {
                m_context->printf_data_func("\b");
            }
            /* Do auto complete */
            AutoComplete(m_context);
            /* Move position to end */
            m_context->c_pos = m_context->l_pos = StrLen(m_context->line);
#endif
            return 0;
        }
#if SHELL_SEARCH_IN_HIST
        /* Search command in history */
        else if ((ch == '`') && (m_context->l_pos == 0) && (m_context->line[0] == 0x00))
        {
        }
#endif
        /* Handle backspace key */
        else if ((ch == KET_DEL) || (ch == '\b'))
        {
            /* There must be at last one char */
            if (m_context->c_pos == 0)
            {
                return 0;
            }

            m_context->l_pos--;
            m_context->c_pos--;

            if (m_context->l_pos > m_context->c_pos)
            {
                memmove(&m_context->line[m_context->c_pos], &m_context->line[m_context->c_pos + 1],
                        m_context->l_pos - m_context->c_pos);
                m_context->line[m_context->l_pos] = 0;
                m_context->printf_data_func("\b%s  \b", &m_context->line[m_context->c_pos]);

                /* Reset position */
                for (i = m_context->c_pos; i <= m_context->l_pos; i++)
                {
                    m_context->printf_data_func("\b");
                }
            }
            else /* Normal backspace operation */
            {
                m_context->printf_data_func("\b \b");
                m_context->line[m_context->l_pos] = 0;
            }
            return 0;
        }
        else
        {
            
        }

        /* Input too long */
        if (m_context->l_pos >= (SHELL_BUFFER_SIZE - 1))
        {
            m_context->l_pos = 0;
        }

        /* Handle end of line, break */
        if ((ch == '\r') || (ch == '\n'))
        {
            m_context->printf_data_func("\r\n");
            ProcessCommand(m_context, m_context->line);
            /* Reset all params */
            m_context->c_pos = m_context->l_pos = 0;
            m_context->hist_current = 0;
            m_context->printf_data_func(m_context->prompt);
            memset(m_context->line, 0, sizeof(m_context->line));
            return 0;
        }

        /* Normal character */
        if (m_context->c_pos < m_context->l_pos)
        {
            memmove(&m_context->line[m_context->c_pos + 1], &m_context->line[m_context->c_pos],
                    m_context->l_pos - m_context->c_pos);
            m_context->line[m_context->c_pos] = ch;
            m_context->printf_data_func("%s", &m_context->line[m_context->c_pos]);
            /* Move the cursor to new position */
            for (i = m_context->c_pos; i < m_context->l_pos; i++)
            {
                m_context->printf_data_func("\b");
            }
        }
        else
        {
            m_context->line[m_context->l_pos] = ch;
            if (m_loop_back)
                m_context->printf_data_func("%c", ch);
        }

        ch = 0;
        m_context->l_pos++;
        m_context->c_pos++;
    }

    return 0;
}

static int32_t HelpCommand(p_shell_context_t context, int32_t argc, char **argv)
{
    uint8_t i = 0;

    for (i = 0; i < g_RegisteredCommands.numberOfCommandInList; i++)
    {
        context->printf_data_func(g_RegisteredCommands.CommandList[i]->pcHelpString);
    }
    return 0;
}

static int32_t ExitCommand(p_shell_context_t context, int32_t argc, char **argv)
{
    /* Skip warning */
    context->printf_data_func("\r\nSHELL exited\r\n");
    context->exit = true;
    return 0;
}

static void ProcessCommand(p_shell_context_t context, const char *cmd)
{
    static const shell_command_context_t *tmpCommand = NULL;
    static const char *tmpCommandString;
    int32_t argc;
    char *argv[SHELL_BUFFER_SIZE];
    uint8_t flag = 1;
    uint8_t tmpCommandLen;
    uint8_t tmpLen;
    uint8_t i = 0;

    tmpLen = StrLen(cmd);
    argc = ParseLine(cmd, tmpLen, argv);

    if ((tmpCommand == NULL) && (argc > 0))
    {
        for (i = 0; i < g_RegisteredCommands.numberOfCommandInList; i++)
        {
            tmpCommand = g_RegisteredCommands.CommandList[i];
            tmpCommandString = tmpCommand->pcCommand;
            tmpCommandLen = StrLen(tmpCommandString);
            /* Compare with space or end of string */
            if ((cmd[tmpCommandLen] == ' ') || (cmd[tmpCommandLen] == 0x00))
            {
                if (StrCompare(tmpCommandString, argv[0], tmpCommandLen) == 0)
                {
                    if ((tmpCommand->cExpectedNumberOfParameters == 0) && (argc == 1))
                    {
                        flag = 0;
                    }
                    else if (tmpCommand->cExpectedNumberOfParameters > 0)
                    {
                        if ((argc - 1) == tmpCommand->cExpectedNumberOfParameters)
                        {
                            flag = 0;
                        }
                    }
                    else
                    {
                        flag = 1;
                    }
                    break;
                }
            }
        }
    }

    if ((tmpCommand != NULL) && (flag == 1U))
    {
        context->printf_data_func(
            "\r\nIncorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n\r\n");
        tmpCommand = NULL;
    }
    else if (tmpCommand != NULL)
    {
        tmpLen = StrLen(cmd);
        /* Compare with last command. Push back to history buffer if different */
        if (tmpLen != StrCompare(cmd, context->hist_buf[0], StrLen(cmd)))
        {
            for (i = SHELL_HIST_MAX - 1; i > 0; i--)
            {
                memset(context->hist_buf[i], '\0', SHELL_BUFFER_SIZE);
                tmpLen = StrLen(context->hist_buf[i - 1]);
                StrCopy(context->hist_buf[i], context->hist_buf[i - 1], tmpLen);
            }
            memset(context->hist_buf[0], '\0', SHELL_BUFFER_SIZE);
            tmpLen = StrLen(cmd);
            StrCopy(context->hist_buf[0], cmd, tmpLen);
            if (context->hist_count < SHELL_HIST_MAX)
            {
                context->hist_count++;
            }
        }
        tmpCommand->pFuncCallBack(context, argc, argv);
        tmpCommand = NULL;
    }
    else
    {
        // context->printf_data_func(
        //     "\r\nCommand not recognised.  Enter 'help' to view a list of available commands.\r\n\r\n");
        tmpCommand = NULL;
    }
}

static void GetHistoryCommand(p_shell_context_t context, uint8_t hist_pos)
{
    uint8_t i;
    uint32_t tmp;

    if (context->hist_buf[0][0] == '\0')
    {
        context->hist_current = 0;
        return;
    }
    if (hist_pos > SHELL_HIST_MAX)
    {
        hist_pos = SHELL_HIST_MAX - 1;
    }
    tmp = StrLen(context->line);
    /* Clear current if have */
    if (tmp > 0)
    {
        memset(context->line, '\0', tmp);
        for (i = 0; i < tmp; i++)
        {
            context->printf_data_func("\b \b");
        }
    }

    context->l_pos = StrLen(context->hist_buf[hist_pos]);
    context->c_pos = context->l_pos;
    StrCopy(context->line, context->hist_buf[hist_pos], context->l_pos);
    context->printf_data_func(context->hist_buf[hist_pos]);
}

static void AutoComplete(p_shell_context_t context)
{
    int32_t len;
    int32_t minLen;
    uint8_t i = 0;
    const shell_command_context_t *tmpCommand = NULL;
    const char *namePtr;
    const char *cmdName;

    minLen = 0;
    namePtr = NULL;

    if (!StrLen(context->line))
    {
        return;
    }
    context->printf_data_func("\r\n");
    /* Empty tab, list all commands */
    if (context->line[0] == '\0')
    {
        HelpCommand(context, 0, NULL);
        return;
    }
    /* Do auto complete */
    for (i = 0; i < g_RegisteredCommands.numberOfCommandInList; i++)
    {
        tmpCommand = g_RegisteredCommands.CommandList[i];
        cmdName = tmpCommand->pcCommand;
        if (StrCompare(context->line, cmdName, StrLen(context->line)) == 0)
        {
            if (minLen == 0)
            {
                namePtr = cmdName;
                minLen = StrLen(namePtr);
                /* Show possible matches */
                context->printf_data_func("%s\r\n", cmdName);
                continue;
            }
            len = StrCompare(namePtr, cmdName, StrLen(namePtr));
            if (len < 0)
            {
                len = len * (-1);
            }
            if (len < minLen)
            {
                minLen = len;
            }
        }
    }
    /* Auto complete string */
    if (namePtr)
    {
        StrCopy(context->line, namePtr, minLen);
    }
    context->printf_data_func("%s%s", context->prompt, context->line);
    return;
}

static char *StrCopy(char *dest, const char *src, int32_t count)
{
    char *ret = dest;
    int32_t i = 0;

    for (i = 0; i < count; i++)
    {
        dest[i] = src[i];
    }

    return ret;
}

static int32_t StrLen(const char *str)
{
    int32_t i = 0;

    while (*str)
    {
        str++;
        i++;
    }
    return i;
}

static int32_t StrCompare(const char *str1, const char *str2, int32_t count)
{
    while (count--)
    {
        if (*str1++ != *str2++)
        {
            return *(unsigned char *)(str1 - 1) - *(unsigned char *)(str2 - 1);
        }
    }
    return 0;
}

static int32_t ParseLine(const char *cmd, uint32_t len, char *argv[SHELL_MAX_ARGS])
{
    uint32_t argc;
    char *p;
    uint32_t position;

    /* Init params */
    memset(g_paramBuffer, '\0', len + 1);
    StrCopy(g_paramBuffer, cmd, len);

    p = g_paramBuffer;
    position = 0;
    argc = 0;

    while (position < len)
    {
        /* Skip all blanks */
        while (((char)(*p) == ' ') && (position < len))
        {
            *p = '\0';
            p++;
            position++;
        }
        /* Process begin of a string */
        if (*p == '"')
        {
            p++;
            position++;
            argv[argc] = p;
            argc++;
            /* Skip this string */
            while ((*p != '"') && (position < len))
            {
                p++;
                position++;
            }
            /* Skip '"' */
            *p = '\0';
            p++;
            position++;
        }
        else /* Normal char */
        {
            argv[argc] = p;
            argc++;
            while (((char)*p != ' ') && ((char)*p != '\t') && (position < len))
            {
                p++;
                position++;
            }
        }
    }
    return argc;
}

int32_t SHELL_RegisterCommand(const shell_command_context_t *command_context)
{
    int32_t result = 0;

    /* If have room  in command list */
    if (g_RegisteredCommands.numberOfCommandInList < SHELL_MAX_CMD)
    {
        g_RegisteredCommands.CommandList[g_RegisteredCommands.numberOfCommandInList++] = command_context;
    }
    else
    {
        result = -1;
    }
    return result;
}

static uint8_t GetChar(p_shell_context_t context)
{
    uint8_t ch;

#if SHELL_USE_FILE_STREAM
    ch = fgetc(context->STDIN);
#else
    context->recv_data_func(&ch, 1U);
#endif
    return ch;
}
