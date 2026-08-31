#ifndef FLUIDNC_H
#define FLUIDNC_H
#include <stdint.h>
#include <stdbool.h>

typedef enum { FNC_UNKNOWN, FNC_IDLE, FNC_RUN, FNC_HOLD, FNC_JOG,
               FNC_HOME, FNC_ALARM, FNC_CHECK, FNC_DOOR } fnc_state_e;

typedef struct {
  fnc_state_e state;
  float   mpos[3], wpos[3], wco[3];
  uint16_t feed, speed;
  uint8_t  pins;
  char     sd_file[32];
  uint8_t  sd_pct;
  char     msg[48];
  uint8_t  msg_id;
  char     ip[20];
  uint32_t last_rx;
  bool     connected;
} fnc_data_t;

typedef struct { char name[48]; uint32_t size; } fnc_file_t;

void fnc_init(void);
void fnc_poll(uint32_t now);
const fnc_data_t* fnc_get(void);
void fnc_sd_list(fnc_file_t *dst, uint8_t max, uint8_t *cnt);
void fnc_cmd(const char *s);
void fnc_rt(uint8_t b);
/* Лічильник отриманих "ok" з моменту старту - зростає монотонно.
 * Дозволяє надійно перевірити "чи підтверджені N надісланих команд",
 * не покладаючись на випадковість опитування статусу ('?' раз на 150мс),
 * яке може просто не встигнути зловити короткий/швидкий рух у стані RUN. */
uint32_t fnc_ok_count(void);
void fnc_jog(char axis, const char *dist, uint16_t feed);
void fnc_jog_cancel(void);
void fnc_run_sd(const char *name);
void fnc_clear_msg(void);

/* Перехоплення сирих рядків з UART (для парсингу вмісту файлу через $SD/Show).
 * Поки sniffer != NULL, кожен вхідний рядок іде ТІЛЬКИ в нього, звичайний
 * розбір статусів/повідомлень призупиняється. cb отримує мутабельний буфер. */
typedef void (*fnc_sniffer_cb)(char *line);
void fnc_set_sniffer(fnc_sniffer_cb cb);
/* Призупиняє періодичний '?' опит під час довгого стріму (щоб не влазив
 * статус-рядок посеред виводу файлу). */
void fnc_set_scanning(bool on);
#endif
