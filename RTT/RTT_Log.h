
#ifndef _RTT_LOG_H_
#define _RTT_LOH_H_
#include "SEGGER_RTT.h"

#define LOG_DEBUG 1

#if LOG_DEBUG


#define LOG_PROTO(type,color,format,...)            \
        SEGGER_RTT_printf(0,"%s%s"format"%s", \
                          color,                    \
                          type,                     \
                          ##__VA_ARGS__,            \
                          RTT_CTRL_RESET)

/* 清屏*/
#define LOG_CLEAR() SEGGER_RTT_WriteString(0, "  "RTT_CTRL_CLEAR)

/* 无颜色日志输出 */
#define LOG(format,...) LOG_PROTO("","",format,##__VA_ARGS__)

/* 有颜色格式日志输出 */
#define LOG_GREEN(format,...)      LOG_PROTO("", RTT_CTRL_TEXT_BRIGHT_GREEN , format, ##__VA_ARGS__)
#define LOG_YELLOW(format,...)     LOG_PROTO("", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
#define LOG_RED(format,...)        LOG_PROTO("", RTT_CTRL_TEXT_BRIGHT_RED   , format, ##__VA_ARGS__)
#define LOG_BLUE(format,...)       LOG_PROTO("", RTT_CTRL_TEXT_BRIGHT_BLUE  , format, ##__VA_ARGS__)
#define LOG_MAGENTA(format,...)    LOG_PROTO("", RTT_CTRL_TEXT_BRIGHT_MAGENTA  , format, ##__VA_ARGS__)
#define LOG_CYAN(format,...)       LOG_PROTO("", RTT_CTRL_TEXT_BRIGHT_CYAN  , format, ##__VA_ARGS__)
#define LOG_BLACK(format,...)       LOG_PROTO("", RTT_CTRL_TEXT_BRIGHT_BLACK  , format, ##__VA_ARGS__)
#define LOG_WHITE(format,...)       LOG_PROTO("", RTT_CTRL_TEXT_BRIGHT_WHITE  , format, ##__VA_ARGS__)

#else
#define LOG_CLEAR()
#define LOG
#define LOGI
#define LOGW
#define LOGE

#endif

#endif // !_LOG_H_
