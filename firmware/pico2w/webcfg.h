#ifndef WEBCFG_H
#define WEBCFG_H
#include <stdbool.h>

/* Provided by the firmware: fill the state JSON / apply a JSON request. */
typedef struct {
    void (*state_json)(char *buf, int max);
    void (*apply_json)(const char *body, char *resp, int max);
} webcfg_hooks_t;

void webcfg_init(const webcfg_hooks_t *hooks);   /* main loop, once */
void webcfg_poll(void);                          /* main loop, often */
void webcfg_usb_service(void);
void webcfg_debug(char *buf, int max);          /* status line for the debug log */                   /* TinyUSB context (after tud_task) */

bool webcfg_json_num(const char *json, const char *key, float *out);
bool webcfg_json_str(const char *json, const char *key, char *out, int max);
#endif
