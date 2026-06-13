#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__

/* 注释此行以关闭生产环境调试输出 */
#define DEBUG_LOG_ENABLE

void debug_log_init(void);

#ifdef DEBUG_LOG_ENABLE
void debug_log_print(void);
#else
#define debug_log_print() ((void)0)
#endif

#endif /* __DEBUG_LOG_H__ */
