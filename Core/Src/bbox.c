#include "bbox.h"
#include "fluidnc.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/* Двофазне сканування.
 *
 * ФАЗА PH_PROBE: $ESP222=<N>,sd/<ім'я> - патч FluidNC (додає alias ESP222
 * до вже наявної fileShowSome), читає лише перші N рядків файлу і віддає
 * JSON {"file_lines":["...",";Bounds:...","..."],"path":"...",...}\nok
 * Миттєво, без блокування ESP32, без Soft Reset.
 *
 * ПЕРЕВІРЕНО НА ЗАЛІЗІ (пряме підключення термінала): відповідь приходить
 * НЕ одним рядком, а нарізана на фрагменти виду [JSON:<шматок>], кожен
 * зі своїм переносом рядка - і сам "; Bounds:" цілком міг розірватись
 * рівно посередині значення між двома фрагментами. Тому фрагменти треба
 * зшивати (jbuf/jlen) в один буфер, знімаючи обгортку "[JSON:" / "]", і
 * шукати "Bounds:" вже в зібраному тексті. Ознака останнього фрагмента -
 * вміст перед завершальним "]" закінчується на "}" (закриття JSON-об'єкта).
 * Наступний рядок після цього - звичайний "ok" (без обгортки).
 *
 * ФАЗА PH_FULL (тільки якщо в PH_PROBE заголовка не знайшлось - тобто це
 * не LightBurn-файл): старий шлях через $SD/Show=sd/<ім'я> - повний
 * потоковий модальний парсинг G0-G3. Для маленьких файлів (ArtCAM) це і
 * так вже підтверджено на залізі як робоче й швидке (і без [JSON:]-обгортки
 * - $SD/Show віддає сирі рядки файлу напряму). Великі файли без заголовка
 * гейтяться порогом розміру (BBOX_MAX_FULL_PARSE) - в PH_FULL не пускаються
 * взагалі, щоб не повторити зависання. */

#define BBOX_PROBE_LINES        50
#define HEADER_SAFETY_LINES     300
#define BBOX_TIMEOUT_MS         12000
/* Файл без заголовка "Bounds:" (не LightBurn) вище цього розміру НЕ
 * пускаємо в PH_FULL ($SD/Show повний парсинг) - консервативний поріг,
 * підбирайте за фактом тестування. Підтверджено: 20МБ вішає FluidNC
 * повністю; є відомі issue в репо FluidNC, де це траплялось навіть на
 * ~230КБ на деяких версіях/платах. Малі ArtCAM-файли (десятки КБ) вже
 * підтверджено безпечні. */
#define BBOX_MAX_FULL_PARSE     (100UL * 1024UL)
/* Буфер для зшивання фрагментів [JSON:...] в один текст перед пошуком
 * Bounds/G91. 50 рядків файлу, навіть щільних растрових (як у прикладі
 * з реального 20МБ файлу), в JSON з лапками/комами вкладаються з запасом. */
#define JBUF_SIZE               4096

typedef enum { PH_PROBE, PH_FULL } bbox_phase_e;

static bbox_t   st;
static bbox_phase_e phase;
static int      gmode;          /* -1 = ще не було жодного G0..G3 (PH_FULL) */
static float    curx, cury;
static uint16_t line_no;
static bool     got_point;
static bool     in_header;      /* PH_FULL: усі рядки поки що коментарі (';') */
static uint32_t scan_t0;
static uint32_t file_size;      /* розмір файлу з SD-лістингу - для порогу PH_FULL */
static char     fname[48];      /* нормалізоване ім'я - для fallback-команди PH_FULL (розмір як fnc_file_t.name) */
static char     jbuf[JBUF_SIZE];/* зшитий текст усіх [JSON:...] фрагментів PH_PROBE */
static uint16_t jlen;

/* need_abort=true лише коли реально уриваємо стрім, що ще триває сам по
 * собі (таймаут PH_FULL). $ESP222 у PH_PROBE в цьому не потребує взагалі -
 * він і так читає рівно N рядків і сам зупиняється. */
static void finish(bbox_state_e final, bool need_abort){
  if(need_abort){
    fnc_rt('!');
    fnc_rt(0x18);
  }
  fnc_set_sniffer(NULL);
  fnc_set_scanning(false);
  st.state = final;
}

/* "Bounds: X-1.25 Y0.02 to X37.85 Y36.82" -> 4 числа (працює і всередині
 * JSON-рядка: strtof сам стає на першому нецифровому символі, лапка чи
 * кома для нього так само "кінець числа", як і пробіл чи 'Y'/'X'). */
static bool parse_bounds(const char *p, float *x1, float *y1, float *x2, float *y2){
  char *e;
  const char *q = strchr(p, 'X');
  if(!q) return false;
  *x1 = strtof(q+1, &e); if(e == q+1) return false;
  q = strchr(e, 'Y'); if(!q) return false;
  *y1 = strtof(q+1, &e); if(e == q+1) return false;
  q = strchr(e, 'X'); if(!q) return false;
  *x2 = strtof(q+1, &e); if(e == q+1) return false;
  q = strchr(e, 'Y'); if(!q) return false;
  *y2 = strtof(q+1, &e); if(e == q+1) return false;
  return true;
}

/* LightBurn-файли на цьому верстаті вмикають G91 (відносний режим) одразу
 * після заголовка - реальний job виконується відносно точки старту, а не
 * абсолютних координат. Це означає: "; Bounds:" - це розмах ВІДНОСНО
 * точки старту, а не абсолютні X/Y. Виключає G91.1 (режим IJK для дуг -
 * інший код) і гіпотетичні G910+ (крапка чи не-цифра одразу після). */
static bool has_g91(const char *s){
  const char *p = s;
  while((p = strstr(p, "G91")) != NULL){
    char after = p[3];
    if((after < '0' || after > '9') && after != '.') return true;
    p += 3;
  }
  return false;
}

static bool try_bounds(const char *raw){
  const char *b = strstr(raw, "Bounds:");
  if(!b) return false;
  float x1, y1, x2, y2;
  if(!parse_bounds(b, &x1, &y1, &x2, &y2)) return false;
  st.minx = (x1 < x2) ? x1 : x2;  st.maxx = (x1 > x2) ? x1 : x2;
  st.miny = (y1 < y2) ? y1 : y2;  st.maxy = (y1 > y2) ? y1 : y2;
  st.from_header = true;
  st.relative = has_g91(raw);
  return true;
}

/* Вирізає коментарі в рядку: все після ';' і все між '(' та ')'. */
static void strip_comments(char *l){
  char *sc = strchr(l, ';');
  if(sc) *sc = 0;
  char *p = l;
  while((p = strchr(p, '(')) != NULL){
    char *end = strchr(p, ')');
    if(!end){ *p = 0; break; }
    memmove(p, end+1, strlen(end+1)+1);
  }
}

/* Модальний токенізер: G-слово і осі повторюються лише коли змінюються
 * (ArtCAM-стиль: "X18.024Y-11.175" на одному рядку, або просто "Y-12.744"
 * саме по собі — успадковує попередній режим руху і попередню X). */
static void tokenize(char *l){
  char *p = l;
  bool moved = false;
  while(*p){
    if(isalpha((unsigned char)*p)){
      char letter = (char)toupper((unsigned char)*p++);
      char *e;
      float v = strtof(p, &e);
      if(e != p){
        switch(letter){
          case 'G': if(v==0.0f||v==1.0f||v==2.0f||v==3.0f) gmode = (int)v; break;
          case 'X': curx = v; moved = true; break;
          case 'Y': cury = v; moved = true; break;
          default: break; /* Z,F,S,T,M,I,J - ігноруємо для bbox */
        }
        p = e;
      } else p++;
    } else p++;
  }
  if(moved && gmode >= 0){
    if(curx < st.minx) st.minx = curx;
    if(curx > st.maxx) st.maxx = curx;
    if(cury < st.miny) st.miny = cury;
    if(cury > st.maxy) st.maxy = cury;
    got_point = true;
  }
}

static void start_full_fallback(void){
  phase = PH_FULL;
  in_header = true;
  gmode = -1; curx = 0; cury = 0;
  char b[64];
  snprintf(b, sizeof(b), "$SD/Show=sd/%s", fname);
  fnc_cmd(b);
}

/* Знімає обгортку "[JSON:" ... "]" з одного фрагмента. is_last=true, якщо
 * вміст перед завершальним ']' закінчується на '}' (закриття JSON-об'єкта -
 * ознака останнього фрагмента багаточастинної відповіді). */
static bool strip_json_frag(const char *raw, const char **content, size_t *len, bool *is_last){
  if(strncmp(raw, "[JSON:", 6) != 0) return false;
  size_t rl = strlen(raw);
  if(rl < 7 || raw[rl-1] != ']') return false;
  *content = raw + 6;
  *len = rl - 7;
  *is_last = (*len > 0) && ((*content)[*len-1] == '}');
  return true;
}

static void bbox_line(char *raw){
  line_no++;
  st.lines = line_no;

  /* LightBurn/Windows шле CRLF — прибираємо кінцевий '\r'. */
  size_t rl = strlen(raw);
  if(rl && raw[rl-1] == '\r') raw[rl-1] = 0;

  if(phase == PH_PROBE){
    const char *content; size_t clen; bool is_last;
    if(strip_json_frag(raw, &content, &clen, &is_last)){
      /* Зшиваємо фрагмент у jbuf (з відсіканням, якщо раптом не влізе -
       * безпечна деградація: просто не знайдемо Bounds і підемо far fallback). */
      if(jlen + clen < sizeof(jbuf) - 1){
        memcpy(jbuf + jlen, content, clen);
        jlen = (uint16_t)(jlen + clen);
        jbuf[jlen] = 0;
      }
      if(is_last) try_bounds(jbuf); /* результат ляже в st.from_header/relative/minx.. */
      return;
    }
    if(!strcmp(raw, "ok") || !strncmp(raw, "error", 5)){
      /* $ESP222 завершився. Знайшли Bounds у зібраному JSON - вже готово. */
      if(st.from_header){ finish(BBOX_DONE, false); return; }
      /* Не знайшли. Якщо файл малий - це, найімовірніше, ArtCAM без
       * заголовка, і повний $SD/Show для нього вже підтверджено безпечний.
       * Якщо великий - заголовка Bounds нема (не LightBurn), і повний
       * парсинг ризикує повторити зависання на 20МБ-файлах - не пускаємо. */
      if(file_size > BBOX_MAX_FULL_PARSE){ finish(BBOX_TOO_BIG, false); return; }
      start_full_fallback();
      return;
    }
    return; /* невідомий формат рядка в PH_PROBE - ігноруємо */
  }

  /* PH_FULL - як і раніше: ok/error, заголовок (про всяк випадок, раптом
   * трапиться поза вікном перших 50 рядків), інакше токенізація. */
  if(!strcmp(raw, "ok")){
    finish(got_point || st.from_header ? BBOX_DONE : BBOX_ERROR, false);
    return;
  }
  if(!strncmp(raw, "error", 5)){
    finish(BBOX_ERROR, false);
    return;
  }
  if(in_header){
    if(raw[0] == ';'){
      if(try_bounds(raw)){ finish(BBOX_DONE, true); return; }
      if(line_no > HEADER_SAFETY_LINES) in_header = false;
    } else {
      in_header = false;
    }
  }
  strip_comments(raw);
  tokenize(raw);
}

void bbox_start(const char *sd_name, uint32_t size){
  memset(&st, 0, sizeof(st));
  st.minx = st.miny = 1e9f;
  st.maxx = st.maxy = -1e9f;
  line_no = 0; got_point = false;
  scan_t0 = HAL_GetTick();
  st.state = BBOX_SCANNING;
  phase = PH_PROBE;
  file_size = size;
  jlen = 0; jbuf[0] = 0;

  const char *nm = sd_name;
  while(*nm == '/' || *nm == ' ') nm++;
  snprintf(fname, sizeof(fname), "%s", nm);

  char b[64];
  snprintf(b, sizeof(b), "$ESP222=%u,sd/%s", (unsigned)BBOX_PROBE_LINES, fname);

  fnc_set_sniffer(bbox_line);
  fnc_set_scanning(true);
  fnc_cmd(b);
}

void bbox_tick(uint32_t now){
  if(st.state == BBOX_SCANNING && (now - scan_t0 > BBOX_TIMEOUT_MS)){
    finish(got_point ? BBOX_DONE : BBOX_ERROR, true);
  }
}

void bbox_cancel(void){
  if(st.state == BBOX_SCANNING) finish(BBOX_IDLE, true);
}

const bbox_t* bbox_get(void){ return &st; }
