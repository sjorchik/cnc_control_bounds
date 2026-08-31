#ifndef SETTINGS_H
#define SETTINGS_H
#include <stdint.h>

typedef struct {
  uint16_t feed;
  uint8_t  jog_mode;
  uint8_t  profile;      /* 0 = SPINDLE, 1 = LASER */
  uint8_t  power[2];     /* % потужності профілів */
  uint16_t trace_feed;   /* мм/хв для об'їзду периметру SD-файлу */
  uint8_t  trace_power;  /* % потужності об'їзду (лише LASER) */
} settings_t;

void settings_load(settings_t *s);
void settings_save(const settings_t *s);
#endif
