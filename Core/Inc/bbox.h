#ifndef BBOX_H
#define BBOX_H
#include <stdint.h>
#include <stdbool.h>

typedef enum { BBOX_IDLE, BBOX_SCANNING, BBOX_DONE, BBOX_ERROR, BBOX_TOO_BIG } bbox_state_e;

typedef struct {
  bbox_state_e state;
  float    minx, maxx, miny, maxy;
  uint32_t lines;       /* к-сть оброблених рядків — для індикації активності */
  bool     from_header; /* bbox взято з "; Bounds:" (LightBurn), а не з повного парсингу */
  bool     relative;    /* файл використовує G91 - Bounds це розмах ВІДНОСНО точки
                          * старту завдання, а не абсолютні координати верстата */
} bbox_t;

/* Запускає сканування файлу з SD (ім'я як у fnc_file_t.name, розмір як у
 * fnc_file_t.size). Неблокуюче — результат перевіряти через
 * bbox_get()/bbox_tick(). Якщо в перших рядках немає заголовка "Bounds:"
 * (не LightBurn), а розмір файлу перевищує безпечний поріг для повного
 * $SD/Show-парсингу — сканування одразу завершиться станом BBOX_TOO_BIG,
 * а не піде довгим/ризикованим шляхом (великі SD-читання без заголовка
 * підтверджено вішають FluidNC). */
void bbox_start(const char *sd_name, uint32_t size);

/* Викликати щотіку з ui_tick(). Стежить за таймаутом. */
void bbox_tick(uint32_t now);

/* Ручне скасування сканування, що триває. */
void bbox_cancel(void);

const bbox_t* bbox_get(void);
#endif
