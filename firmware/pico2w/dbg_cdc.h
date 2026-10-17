#ifndef DBG_CDC_H
#define DBG_CDC_H
#include <stdint.h>
void     dbg_cdc_init(void);     /* after stdio_init_all() */
void     dbg_cdc_task(void);     /* main loop, after tud_task() */
uint32_t dbg_cdc_dropped(void);
void     dbg_logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#endif
