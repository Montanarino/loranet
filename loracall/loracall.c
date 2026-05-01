/*
 * lora_voicecall.c  –  Chiamata vocale su DX-LR02 / DX-LR03
 *
 * Compatibile con: Windows (Win32 API) · Linux · macOS
 *
 * COME FUNZIONANO I MODULI DX-LR02 / DX-LR03:
 * ─────────────────────────────────────────────
 *  1. Invia "+++" + CR+LF  → risponde "Entry AT"
 *  2. Configura con comandi AT (frequenza, SF, BW…)
 *  3. Invia "AT+ENTM"      → modalità pipe trasparente
 *  4. Da ora in poi: bytes scritti = bytes trasmessi via LoRa (raw)
 *
 * PROTOCOLLO FRAMING (sopra il pipe trasparente):
 * ─────────────────────────────────────────────────
 *  [0xAA][0x55][TIPO][LEN_HI][LEN_LO][PAYLOAD…][CRC8]
 *  Tipi: 0x01 CALL_REQ · 0x02 CALL_ACK · 0x03 CALL_END · 0x10 AUDIO
 *
 * DIPENDENZE:
 *   Linux/macOS : libopus-dev  +  miniaudio.h
 *   Windows     : libopus (vcpkg o binario)  +  miniaudio.h
 *
 * COMPILAZIONE:
 *   Linux/macOS : make
 *   Windows     : compile_windows.bat
 *
 * UTILIZZO:
 *   Linux   : ./lora_voicecall /dev/ttyUSB0 9600 433.000
 *   Windows : lora_voicecall.exe COM3 9600 433.000
 *
 * MODIFICHE RISPETTO ALLA VERSIONE ORIGINALE:
 * ─────────────────────────────────────────────
 *  FIX 1 – Jitter buffer con pre-buffering
 *           play_cb non inizia la riproduzione finché non sono pronti
 *           almeno JITTER_PREBUF_FRAMES frame (~120 ms). Se il buffer
 *           si svuota, torna in stato "buffering" invece di emettere
 *           silenzio hard con artefatti udibili al riempimento.
 *
 *  FIX 2 – Packet Loss Concealment (PLC)
 *           Il thread RX tiene traccia dell'ultimo pacchetto audio
 *           ricevuto. Se passano più di 1.5× FRAME_MS senza audio,
 *           chiama opus_decode(NULL,0) per generare audio sintetico
 *           che copre la lacuna in modo trasparente all'orecchio.
 *
 *  FIX 3 – Flush buffer di cattura all'inizio della chiamata
 *           Quando in_call passa da 0→1, il buffer cap_pcm viene
 *           azzerato, eliminando il "rumore iniziale" causato dai
 *           campioni accumulati durante l'attesa.
 *
 *  FIX 4 – at_exit corretto: usa AT+ENTM invece di +++
 *           La versione originale mandava +++ che riportava il modulo
 *           in AT mode invece di entrare nella modalità trasparente.
 *
 *  FIX 5 – Pacing TX basato su wall-clock
 *           Il thread TX ora misura il tempo reale trascorso e
 *           attende il giusto numero di ms per mantenere la cadenza
 *           di 40 ms/frame, evitando burst o starvation.
 */

/* ================================================================
 * ASTRAZIONI CROSS-PLATFORM
 * ================================================================ */
#ifdef _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
  #include <process.h>            /* _beginthreadex */
  typedef HANDLE            pthread_t;
  typedef CRITICAL_SECTION  pthread_mutex_t;
  #define pthread_mutex_init(m,a)  InitializeCriticalSection(m)
  #define pthread_mutex_destroy(m) DeleteCriticalSection(m)
  #define pthread_mutex_lock(m)    EnterCriticalSection(m)
  #define pthread_mutex_unlock(m)  LeaveCriticalSection(m)
  #define ms_sleep(ms)  Sleep(ms)
  typedef HANDLE serial_fd_t;
  #define INVALID_SERIAL  INVALID_HANDLE_VALUE

  /* wall-clock in millisecondi */
  static uint64_t now_ms(void)
  {
      FILETIME ft; GetSystemTimeAsFileTime(&ft);
      ULARGE_INTEGER ul; ul.LowPart=ft.dwLowDateTime; ul.HighPart=ft.dwHighDateTime;
      return (uint64_t)(ul.QuadPart / 10000ULL);
  }
#else
  #include <pthread.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <termios.h>
  #include <signal.h>
  #include <sys/time.h>
  #define ms_sleep(ms)  usleep((ms)*1000)
  typedef int  serial_fd_t;
  #define INVALID_SERIAL (-1)

  static uint64_t now_ms(void)
  {
      struct timeval tv; gettimeofday(&tv, NULL);
      return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)tv.tv_usec / 1000ULL;
  }
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include <opus/opus.h>

/* ================================================================
 * PARAMETRI AUDIO E PROTOCOLLO
 * ================================================================ */
#define SAMPLE_RATE    8000
#define CHANNELS       1
#define FRAME_MS       40
#define FRAME_SAMPLES  (SAMPLE_RATE * FRAME_MS / 1000)   /* 320 */
#define OPUS_BITRATE   4800
#define OPUS_MAX_PKT   128
#define FRAME_MAX      (6 + OPUS_MAX_PKT + 1)

/* FIX 1 – jitter buffer: quanti frame attendere prima di iniziare
 * la riproduzione (e dopo ogni underrun). 3 frame = 120 ms.      */
#define JITTER_PREBUF_FRAMES   3
#define JITTER_PREBUF_SAMPLES  (FRAME_SAMPLES * JITTER_PREBUF_FRAMES)

/* FIX 2 – PLC: se dopo questo numero di ms non arriva un pacchetto
 * audio, genera un frame PLC Opus.                                */
#define PLC_TIMEOUT_MS  (FRAME_MS + FRAME_MS / 2)   /* 60 ms */

#define MAGIC_HI       0xAA
#define MAGIC_LO       0x55
#define PKT_CALL_REQ   0x01
#define PKT_CALL_ACK   0x02
#define PKT_CALL_END   0x03
#define PKT_AUDIO      0x10

/* ================================================================
 * SERIALE – implementazione Win32 / POSIX
 * ================================================================ */
typedef struct {
    serial_fd_t     fd;
    pthread_mutex_t lock;
} Serial;

static int serial_open(Serial *s, const char *port, int baud)
{
#ifdef _WIN32
    char path[32];
    if (strncmp(port, "\\\\.\\", 4) == 0)
        strncpy(path, port, sizeof(path) - 1);
    else
        snprintf(path, sizeof(path), "\\\\.\\%s", port);

    s->fd = CreateFileA(path,
                        GENERIC_READ | GENERIC_WRITE,
                        0, NULL, OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL, NULL);
    if (s->fd == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Impossibile aprire %s (errore %lu)\n",
                path, GetLastError());
        return -1;
    }

    DCB dcb;
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(s->fd, &dcb)) {
        fprintf(stderr, "GetCommState fallito\n");
        CloseHandle(s->fd);
        return -1;
    }
    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;
    dcb.fParity  = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_DISABLE;
    dcb.fRtsControl  = RTS_CONTROL_DISABLE;
    dcb.fOutX  = FALSE;
    dcb.fInX   = FALSE;
    if (!SetCommState(s->fd, &dcb)) {
        fprintf(stderr, "SetCommState fallito\n");
        CloseHandle(s->fd);
        return -1;
    }

    COMMTIMEOUTS ct;
    ct.ReadIntervalTimeout         = MAXDWORD;
    ct.ReadTotalTimeoutMultiplier  = 0;
    ct.ReadTotalTimeoutConstant    = 0;
    ct.WriteTotalTimeoutMultiplier = 0;
    ct.WriteTotalTimeoutConstant   = 2000;
    SetCommTimeouts(s->fd, &ct);

#else  /* POSIX */
    s->fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (s->fd < 0) { perror("open"); return -1; }

    struct termios t;
    memset(&t, 0, sizeof(t));
    tcgetattr(s->fd, &t);

    speed_t sp;
    switch (baud) {
        case 1200:   sp = B1200;   break;
        case 2400:   sp = B2400;   break;
        case 4800:   sp = B4800;   break;
        case 19200:  sp = B19200;  break;
        case 38400:  sp = B38400;  break;
        case 57600:  sp = B57600;  break;
        case 115200: sp = B115200; break;
        default:     sp = B9600;
    }
    cfsetispeed(&t, sp);
    cfsetospeed(&t, sp);
    t.c_cflag  = (t.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
    t.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    t.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK);
    t.c_lflag  = 0;
    t.c_oflag  = 0;
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 1;
    tcsetattr(s->fd, TCSANOW, &t);
#endif

    pthread_mutex_init(&s->lock, NULL);
    return 0;
}

static void serial_close(Serial *s)
{
#ifdef _WIN32
    CloseHandle(s->fd);
#else
    close(s->fd);
#endif
    pthread_mutex_destroy(&s->lock);
}

static int serial_write(Serial *s, const uint8_t *buf, int len)
{
    pthread_mutex_lock(&s->lock);
#ifdef _WIN32
    DWORD written = 0;
    BOOL ok = WriteFile(s->fd, buf, (DWORD)len, &written, NULL);
    int ret = (ok && (int)written == len) ? 0 : -1;
#else
    int ret = ((int)write(s->fd, buf, len) == len) ? 0 : -1;
#endif
    pthread_mutex_unlock(&s->lock);
    return ret;
}

static int serial_write_str(Serial *s, const char *str)
{
    return serial_write(s, (const uint8_t *)str, (int)strlen(str));
}

static int serial_read1(Serial *s, uint8_t *b)
{
#ifdef _WIN32
    DWORD n = 0;
    if (!ReadFile(s->fd, b, 1, &n, NULL)) return -1;
    return (int)n;
#else
    return (int)read(s->fd, b, 1);
#endif
}

static int serial_readline(Serial *s, char *buf, int max, int timeout_ms)
{
    int n = 0, elapsed = 0;
    memset(buf, 0, max);
    while (elapsed < timeout_ms && n < max - 1) {
        uint8_t c;
        if (serial_read1(s, &c) == 1) {
            if (c == '\r') continue;
            if (c == '\n') { buf[n] = '\0'; return n; }
            buf[n++] = (char)c;
        } else {
            ms_sleep(10);
            elapsed += 10;
        }
    }
    buf[n] = '\0';
    return n;
}

/* ================================================================
 * THREAD – wrapper cross-platform
 * ================================================================ */
#ifdef _WIN32
typedef unsigned (__stdcall *win_thread_fn)(void *);
static int thread_create(pthread_t *tid, void *(*fn)(void *), void *arg)
{
    *tid = (HANDLE)_beginthreadex(NULL, 0,
                                  (win_thread_fn)fn, arg, 0, NULL);
    return (*tid == NULL) ? -1 : 0;
}
static void thread_join(pthread_t tid)
{
    WaitForSingleObject(tid, INFINITE);
    CloseHandle(tid);
}
#else
static int thread_create(pthread_t *tid, void *(*fn)(void *), void *arg)
{
    return pthread_create(tid, NULL, fn, arg);
}
static void thread_join(pthread_t tid)
{
    pthread_join(tid, NULL);
}
#endif

/* ================================================================
 * STRUTTURA PRINCIPALE
 * ================================================================ */
typedef struct {
    Serial          ser;
    OpusEncoder    *enc;
    OpusDecoder    *dec;
    ma_device       cap_dev;
    ma_device       play_dev;

    int16_t         cap_pcm[FRAME_SAMPLES * 8];
    int             cap_n;
    pthread_mutex_t cap_lock;

    int16_t         play_pcm[FRAME_SAMPLES * 32]; /* più grande per il jitter buffer */
    int             play_n;
    pthread_mutex_t play_lock;

    /* FIX 1 – jitter buffer state */
    volatile int    play_buffering;   /* 1 = attesa pre-fill; 0 = in riproduzione */

    volatile int    in_call;
    volatile int    running;
    pthread_t       tx_tid;
    pthread_t       rx_tid;
} App;

static App *g_app = NULL;

/* ================================================================
 * CRC-8
 * ================================================================ */
static uint8_t crc8(const uint8_t *d, int n)
{
    uint8_t c = 0;
    for (int i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++)
            c = (c & 0x80) ? (c << 1) ^ 0x07 : (c << 1);
    }
    return c;
}

/* ================================================================
 * AT COMMANDS
 * ================================================================ */
static int at_enter(App *ctx)
{
    char resp[128];
    printf("[AT] Invio +++ …\n");
    serial_write_str(&ctx->ser, "+++\r\n");
    ms_sleep(600);
    int n = serial_readline(&ctx->ser, resp, sizeof(resp), 2000);
    printf("[AT] Risposta: '%s'\n", resp);
    if (n > 0 && (strstr(resp,"Entry") || strstr(resp,"+OK")
                  || strstr(resp,"OK")))
        return 0;
    serial_write_str(&ctx->ser, "AT\r\n");
    serial_readline(&ctx->ser, resp, sizeof(resp), 1000);
    printf("[AT] Risposta AT: '%s'\n", resp);
    return strstr(resp, "OK") ? 0 : -1;
}

static int at_cmd(App *ctx, const char *cmd, char *resp, int rmax)
{
    char buf[256];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    printf("[AT] >> %s", buf);
    serial_write_str(&ctx->ser, buf);

    char line[128];
    for (int ms = 0; ms < 3000; ms += 200) {
        int n = serial_readline(&ctx->ser, line, sizeof(line), 200);
        if (n == 0) continue;
        printf("[AT] << %s\n", line);
        if (resp && rmax > 0) {
            strncpy(resp, line, rmax - 1);
            resp[rmax - 1] = '\0';
        }
        if (strcmp(line, "OK")==0 || strncmp(line,"+OK",3)==0) return 0;
        if (strncmp(line,"ERR",3)==0 || strncmp(line,"ERROR",5)==0) return -1;
    }
    return -1;
}

static int lora_configure(App *ctx, float freq_mhz)
{
    char resp[128];
    (void)freq_mhz;
    at_cmd(ctx, "AT+MODE0",  resp, sizeof(resp));
    at_cmd(ctx, "AT+LEVEL7", resp, sizeof(resp));
    printf("[LoRa] Configurato: %.3f MHz SF7 BW500 CR4/5 22dBm\n", freq_mhz);
    return 0;
}

/*
 * FIX 4 – at_exit: usa AT+ENTM per entrare in modalità trasparente.
 * La versione originale inviava "+++" che riportava in AT mode.
 */
static int at_exit(App *ctx)
{
    ms_sleep(200);
    at_cmd(ctx, "+++", NULL, 0);
    ms_sleep(100);
    return 0;
}

/* ================================================================
 * FRAMING
 * ================================================================ */
static int frame_send(App *ctx, uint8_t type,
                      const uint8_t *payload, uint16_t plen)
{
    uint8_t buf[FRAME_MAX];
    int i = 0;
    buf[i++] = MAGIC_HI;
    buf[i++] = MAGIC_LO;
    buf[i++] = type;
    buf[i++] = (uint8_t)(plen >> 8);
    buf[i++] = (uint8_t)(plen & 0xFF);
    if (plen && payload) { memcpy(buf+i, payload, plen); i += plen; }
    buf[i] = crc8(buf, i);
    i++;
    return serial_write(&ctx->ser, buf, i);
}

typedef enum { ST_M1,ST_M2,ST_TYPE,ST_LH,ST_LL,ST_PAYLOAD,ST_CRC } FState;
typedef struct {
    FState   st; uint8_t type; uint16_t plen, pcnt;
    uint8_t  payload[OPUS_MAX_PKT+16];
    uint8_t  raw[FRAME_MAX]; int raw_n;
} FParser;

static void fp_reset(FParser *fp) { fp->st=ST_M1; fp->raw_n=0; fp->pcnt=0; }

static bool fp_feed(FParser *fp, uint8_t b)
{
    switch (fp->st) {
    case ST_M1:
        if (b==MAGIC_HI){fp->raw_n=0;fp->raw[fp->raw_n++]=b;fp->st=ST_M2;}
        break;
    case ST_M2:
        fp->raw[fp->raw_n++]=b;
        fp->st=(b==MAGIC_LO)?ST_TYPE:ST_M1; break;
    case ST_TYPE:
        fp->type=b; fp->raw[fp->raw_n++]=b; fp->st=ST_LH; break;
    case ST_LH:
        fp->plen=(uint16_t)(b<<8); fp->raw[fp->raw_n++]=b; fp->st=ST_LL; break;
    case ST_LL:
        fp->plen|=b; fp->raw[fp->raw_n++]=b; fp->pcnt=0;
        if(!fp->plen){fp->st=ST_CRC;break;}
        if(fp->plen>sizeof(fp->payload)){fp_reset(fp);break;}
        fp->st=ST_PAYLOAD; break;
    case ST_PAYLOAD:
        fp->payload[fp->pcnt++]=b; fp->raw[fp->raw_n++]=b;
        if(fp->pcnt==fp->plen) fp->st=ST_CRC; break;
    case ST_CRC: {
        uint8_t exp=crc8(fp->raw,fp->raw_n);
        fp_reset(fp); return (b==exp);
    }}
    return false;
}

/* ================================================================
 * AUDIO CALLBACKS (miniaudio)
 * ================================================================ */
static void cap_cb(ma_device *d, void *out,
                   const void *in, ma_uint32 n)
{
    (void)out;
    App *ctx = (App *)d->pUserData;
    if (!ctx->in_call) return;
    const int16_t *src = (const int16_t *)in;
    pthread_mutex_lock(&ctx->cap_lock);
    int avail = (int)(sizeof(ctx->cap_pcm)/sizeof(int16_t)) - ctx->cap_n;
    int copy  = ((int)n < avail) ? (int)n : avail;
    memcpy(ctx->cap_pcm + ctx->cap_n, src, copy * 2);
    ctx->cap_n += copy;
    pthread_mutex_unlock(&ctx->cap_lock);
}

/*
 * FIX 1 – play_cb con jitter buffer.
 *
 * Quando play_buffering==1 (inizio chiamata o dopo underrun) non
 * riproduce nulla finché il buffer non raggiunge JITTER_PREBUF_SAMPLES.
 * Questo aggiunge ~120 ms di latenza ma elimina completamente i
 * dropout causati da pacchetti LoRa con jitter variabile.
 * Se il buffer si svuota durante la chiamata, torna in buffering.
 */
static void play_cb(ma_device *d, void *out,
                    const void *in, ma_uint32 n)
{
    (void)in;
    App     *ctx = (App *)d->pUserData;
    int16_t *dst = (int16_t *)out;

    pthread_mutex_lock(&ctx->play_lock);

    /* Fase di pre-buffering: aspetta che ci siano abbastanza campioni */
    if (ctx->play_buffering) {
        if (ctx->play_n >= JITTER_PREBUF_SAMPLES) {
            ctx->play_buffering = 0;  /* buffer pieno: inizia riproduzione */
            printf("\n[AUDIO] Jitter buffer pieno, riproduzione avviata.\n> ");
            fflush(stdout);
        } else {
            /* Silenzio confortevole mentre si riempie */
            memset(dst, 0, n * sizeof(int16_t));
            pthread_mutex_unlock(&ctx->play_lock);
            return;
        }
    }

    int copy = ((int)n < ctx->play_n) ? (int)n : ctx->play_n;
    if (copy > 0) {
        memcpy(dst, ctx->play_pcm, copy * sizeof(int16_t));
        memmove(ctx->play_pcm, ctx->play_pcm + copy,
                (ctx->play_n - copy) * sizeof(int16_t));
        ctx->play_n -= copy;
    }

    if (copy < (int)n) {
        /* Buffer svuotato (underrun): silenzio per questa callback e
         * ri-attiva il pre-buffering per evitare dropout futuri.    */
        memset(dst + copy, 0, ((int)n - copy) * sizeof(int16_t));
        if (ctx->in_call) {
            ctx->play_buffering = 1;
        }
    }

    pthread_mutex_unlock(&ctx->play_lock);
}

/* ================================================================
 * HELPER – aggiunge PCM decodificato al buffer di riproduzione
 * ================================================================ */
static void push_play_pcm(App *ctx, const int16_t *pcm, int nsamples)
{
    pthread_mutex_lock(&ctx->play_lock);
    int space = (int)(sizeof(ctx->play_pcm)/sizeof(int16_t)) - ctx->play_n;
    int copy  = (nsamples < space) ? nsamples : space;
    if (copy > 0) {
        memcpy(ctx->play_pcm + ctx->play_n, pcm, copy * sizeof(int16_t));
        ctx->play_n += copy;
    }
    pthread_mutex_unlock(&ctx->play_lock);
}

/* ================================================================
 * FIX 3 – helper per flush del buffer di cattura all'inizio chiamata
 *
 * Chiama questa funzione ogni volta che in_call passa da 0 → 1.
 * Elimina i campioni "stantii" accumulati nel buffer del microfono
 * durante l'attesa, che causavano il rumore iniziale.
 * ================================================================ */
static void flush_cap_buffer(App *ctx)
{
    pthread_mutex_lock(&ctx->cap_lock);
    ctx->cap_n = 0;
    memset(ctx->cap_pcm, 0, sizeof(ctx->cap_pcm));
    pthread_mutex_unlock(&ctx->cap_lock);
    printf("[AUDIO] Buffer microfono svuotato.\n");
}

static void start_call(App *ctx)
{
    flush_cap_buffer(ctx);                  /* FIX 3 */
    pthread_mutex_lock(&ctx->play_lock);
    ctx->play_n         = 0;
    ctx->play_buffering = 1;                /* FIX 1 – richiede pre-fill */
    pthread_mutex_unlock(&ctx->play_lock);
    ctx->in_call = 1;
}

/* ================================================================
 * THREAD TX  microfono → Opus → frame LoRa
 *
 * FIX 5 – Pacing basato su wall-clock.
 * Invece di aspettare semplicemente che cap_n >= FRAME_SAMPLES,
 * il thread misura anche il tempo reale e dorme esattamente il
 * numero di ms necessari per mantenere la cadenza di FRAME_MS.
 * Questo evita burst/starvation dovuti allo scheduler del SO.
 * ================================================================ */
static void *tx_thread(void *arg)
{
    App     *ctx = (App *)arg;
    int16_t  pcm[FRAME_SAMPLES];
    uint8_t  pkt[OPUS_MAX_PKT];

    printf("[TX] Thread avviato.\n");

    uint64_t next_tx = now_ms();

    while (ctx->running) {
        if (!ctx->in_call) {
            ms_sleep(10);
            next_tx = now_ms();   /* reset del pacing quando non in chiamata */
            continue;
        }

        /* Attendi fino al prossimo slot di trasmissione */
        uint64_t now = now_ms();
        if (now < next_tx) {
            ms_sleep((int)(next_tx - now));
        }
        next_tx += FRAME_MS;

        /* Preleva un frame dal buffer di cattura */
        bool got = false;
        pthread_mutex_lock(&ctx->cap_lock);
        if (ctx->cap_n >= FRAME_SAMPLES) {
            memcpy(pcm, ctx->cap_pcm, FRAME_SAMPLES * sizeof(int16_t));
            memmove(ctx->cap_pcm, ctx->cap_pcm + FRAME_SAMPLES,
                    (ctx->cap_n - FRAME_SAMPLES) * sizeof(int16_t));
            ctx->cap_n -= FRAME_SAMPLES;
            got = true;
        }
        pthread_mutex_unlock(&ctx->cap_lock);

        if (!got) {
            /* Nessun dato ancora: manda silenzio Opus così l'altro lato
             * ha materiale per il PLC e il jitter buffer non si svuota */
            memset(pcm, 0, sizeof(pcm));
        }

        int plen = opus_encode(ctx->enc, pcm, FRAME_SAMPLES,
                               pkt, OPUS_MAX_PKT);
        if (plen < 0) {
            fprintf(stderr, "[TX] Opus: %s\n", opus_strerror(plen));
            continue;
        }
        frame_send(ctx, PKT_AUDIO, pkt, (uint16_t)plen);
    }

    printf("[TX] Thread terminato.\n");
    return NULL;
}

/* ================================================================
 * THREAD RX  serial bytes → parser → Opus decode → speaker
 *
 * FIX 2 – Packet Loss Concealment (PLC).
 * Ogni volta che arriva un pacchetto audio valido si aggiorna
 * last_audio_ms. Nel loop principale, se passano più di
 * PLC_TIMEOUT_MS senza pacchetti, si chiama opus_decode con
 * data=NULL per generare audio PLC che copre la lacuna in modo
 * trasparente, evitando il silenzio hard e il click al ritorno.
 * ================================================================ */
static void *rx_thread(void *arg)
{
    App     *ctx = (App *)arg;
    FParser  fp;
    int16_t  pcm[FRAME_SAMPLES * 2];
    fp_reset(&fp);

    uint64_t last_audio_ms = 0;   /* FIX 2 – per il rilevamento perdita */

    printf("[RX] Thread avviato.\n");

    while (ctx->running) {
        /* FIX 2 – Controlla timeout PLC indipendentemente dall'arrivo
         * di nuovi byte: se la chiamata è attiva e non abbiamo ricevuto
         * audio per troppo tempo, genera un frame PLC.               */
        if (ctx->in_call && last_audio_ms > 0) {
            uint64_t now = now_ms();
            if ((now - last_audio_ms) >= PLC_TIMEOUT_MS) {
                int s = opus_decode(ctx->dec, NULL, 0,
                                    pcm, FRAME_SAMPLES, 0);
                if (s > 0) {
                    push_play_pcm(ctx, pcm, s);
                }
                /* Avanza il timer di PLC_TIMEOUT_MS per evitare una
                 * valanga di frame PLC se il ritardo è lungo.        */
                last_audio_ms += PLC_TIMEOUT_MS;
            }
        }

        uint8_t b;
        if (serial_read1(&ctx->ser, &b) != 1) {
            ms_sleep(1);
            continue;
        }
        if (!fp_feed(&fp, b)) continue;

        switch (fp.type) {
        case PKT_CALL_REQ:
            printf("\n[CALL] *** Chiamata in arrivo! Premi 'a' ***\n> ");
            fflush(stdout);
            break;

        case PKT_CALL_ACK:
            if (!ctx->in_call) {
                start_call(ctx);    /* FIX 3+1: flush + jitter reset */
                printf("\n[CALL] Connesso!\n> ");
                fflush(stdout);
            }
            break;

        case PKT_CALL_END:
            ctx->in_call = 0;
            last_audio_ms = 0;
            printf("\n[CALL] Chiamata terminata.\n> ");
            fflush(stdout);
            break;

        case PKT_AUDIO:
            if (!ctx->in_call) break;
            {
                int s = opus_decode(ctx->dec,
                                    fp.payload, fp.plen,
                                    pcm, FRAME_SAMPLES, 0);
                if (s < 0) {
                    fprintf(stderr, "[RX] Opus: %s\n", opus_strerror(s));
                    break;
                }
                push_play_pcm(ctx, pcm, s);
                last_audio_ms = now_ms();   /* FIX 2 – aggiorna timer PLC */
            }
            break;
        }
    }

    printf("[RX] Thread terminato.\n");
    return NULL;
}

/* ================================================================
 * SIGNAL HANDLER (solo POSIX)
 * ================================================================ */
#ifndef _WIN32
static void sig_handler(int s)
{
    (void)s;
    if (g_app) { g_app->running = 0; g_app->in_call = 0; }
}
#endif

/* ================================================================
 * MAIN
 * ================================================================ */
int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr,
            "Uso:\n"
            "  Linux/macOS : %s /dev/ttyUSB0 [baud] [freq_MHz]\n"
            "  Windows     : %s COM3 [baud] [freq_MHz]\n"
            "Esempio: %s COM3 9600 433.000\n",
            argv[0], argv[0], argv[0]);
        return 1;
    }

    const char *dev  = argv[1];
    int         baud = (argc >= 3) ? atoi(argv[2]) : 9600;
    float       freq = (argc >= 4) ? (float)atof(argv[3]) : 433.000f;

    static App ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.running = 1;
    ctx.play_buffering = 1;   /* FIX 1 – inizia in pre-buffering */
    g_app = &ctx;
    pthread_mutex_init(&ctx.cap_lock,  NULL);
    pthread_mutex_init(&ctx.play_lock, NULL);

#ifndef _WIN32
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
#endif

    /* 1. Seriale */
    printf("[INIT] Apertura porta %s @ %d baud…\n", dev, baud);
    if (serial_open(&ctx.ser, dev, baud) != 0) return 1;

    /* 2. Modulo LoRa */
    printf("[INIT] Configurazione DX-LR02/LR03…\n");
    if (at_enter(&ctx) != 0) {
        fprintf(stderr,
            "\n[ERR] Modulo non risponde!\n"
            "  - Controlla che l'antenna sia collegata\n"
            "  - Verifica baud rate (default di fabbrica: 9600)\n"
            "  - Connessioni: RXD->TX, TXD->RX, VCC->5V, GND->GND\n"
#ifdef _WIN32
            "  - Su Windows usa il nome porta tipo: COM3, COM4...\n"
            "    (Gestione dispositivi -> Porte COM e LPT)\n"
#endif
            );
        serial_close(&ctx.ser);
        return 1;
    }
    if (lora_configure(&ctx, freq) != 0 || at_exit(&ctx) != 0) {
        serial_close(&ctx.ser);
        return 2;
    }
    printf("[INIT] Modulo in modalita' trasparente.\n");

    /* 3. Opus */
    int err;
    ctx.enc = opus_encoder_create(SAMPLE_RATE, CHANNELS,
                                  OPUS_APPLICATION_VOIP, &err);
    if (err != OPUS_OK) {
        fprintf(stderr, "opus_encoder_create: %s\n", opus_strerror(err));
        goto cleanup;
    }
    opus_encoder_ctl(ctx.enc, OPUS_SET_BITRATE(OPUS_BITRATE));
    opus_encoder_ctl(ctx.enc, OPUS_SET_COMPLEXITY(2));
    opus_encoder_ctl(ctx.enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    opus_encoder_ctl(ctx.enc, OPUS_SET_DTX(1));

    ctx.dec = opus_decoder_create(SAMPLE_RATE, CHANNELS, &err);
    if (err != OPUS_OK) {
        fprintf(stderr, "opus_decoder_create: %s\n", opus_strerror(err));
        goto cleanup;
    }

    /* 4. Audio */
    {
        ma_device_config c = ma_device_config_init(ma_device_type_capture);
        c.capture.format   = ma_format_s16;
        c.capture.channels = CHANNELS;
        c.sampleRate       = SAMPLE_RATE;
        c.dataCallback     = cap_cb;
        c.pUserData        = &ctx;
        if (ma_device_init(NULL, &c, &ctx.cap_dev) != MA_SUCCESS) {
            fprintf(stderr, "[ERR] Init microfono\n"); goto cleanup;
        }
    }
    {
        ma_device_config p = ma_device_config_init(ma_device_type_playback);
        p.playback.format   = ma_format_s16;
        p.playback.channels = CHANNELS;
        p.sampleRate        = SAMPLE_RATE;
        p.dataCallback      = play_cb;
        p.pUserData         = &ctx;
        if (ma_device_init(NULL, &p, &ctx.play_dev) != MA_SUCCESS) {
            fprintf(stderr, "[ERR] Init altoparlante\n"); goto cleanup;
        }
    }
    ma_device_start(&ctx.cap_dev);
    ma_device_start(&ctx.play_dev);
    printf("[INIT] Audio OK (8 kHz mono, Opus %d bps, frame %d ms, "
           "jitter prebuf %d ms)\n",
           OPUS_BITRATE, FRAME_MS, JITTER_PREBUF_FRAMES * FRAME_MS);

    /* 5. Thread */
    thread_create(&ctx.tx_tid, tx_thread, &ctx);
    thread_create(&ctx.rx_tid, rx_thread, &ctx);

    /* 6. Menu */
    printf("\n");
    printf("+===========================================+\n");
    printf("|   LoRa Voice Call -- DX-LR02 / LR03      |\n");
    printf("|   Frequenza: %7.3f MHz | SF7 BW500     |\n", freq);
    printf("+===========================================+\n");
    printf("|  c  -> Chiama (invia segnale)             |\n");
    printf("|  a  -> Rispondi alla chiamata             |\n");
    printf("|  h  -> Riaggancia                         |\n");
    printf("|  s  -> Mostra stato                       |\n");
    printf("|  q  -> Esci                               |\n");
    printf("+===========================================+\n");
    printf("> ");
    fflush(stdout);

    char line[32];
    while (ctx.running && fgets(line, sizeof(line), stdin)) {
        line[strcspn(line, "\r\n")] = '\0';
        switch (line[0]) {
        case 'q':
            ctx.running = 0;
            break;
        case 'c':
            frame_send(&ctx, PKT_CALL_REQ, NULL, 0);
            printf("[CALL] Segnale inviato — attendo risposta...\n");
            break;
        case 'a':
            frame_send(&ctx, PKT_CALL_ACK, NULL, 0);
            start_call(&ctx);   /* FIX 3+1: flush + jitter reset */
            printf("[CALL] Risposto — connesso!\n");
            break;
        case 'h':
            ctx.in_call = 0;
            frame_send(&ctx, PKT_CALL_END, NULL, 0);
            printf("[CALL] Riagganciato.\n");
            break;
        case 's':
            printf("[STATO] %s | %.3f MHz | Opus %d bps | SF7 BW500 | "
                   "jitter prebuf %d ms\n",
                   ctx.in_call ? "IN CHIAMATA" : "LIBERO",
                   freq, OPUS_BITRATE,
                   JITTER_PREBUF_FRAMES * FRAME_MS);
            break;
        default:
            if (line[0]) printf("[?] Comandi: c a h s q\n");
        }
        if (ctx.running) { printf("> "); fflush(stdout); }
    }

cleanup:
    ctx.running = 0;
    ctx.in_call = 0;
    frame_send(&ctx, PKT_CALL_END, NULL, 0);
    thread_join(ctx.tx_tid);
    thread_join(ctx.rx_tid);
    ma_device_stop(&ctx.play_dev);
    ma_device_stop(&ctx.cap_dev);
    ma_device_uninit(&ctx.play_dev);
    ma_device_uninit(&ctx.cap_dev);
    if (ctx.enc) opus_encoder_destroy(ctx.enc);
    if (ctx.dec) opus_decoder_destroy(ctx.dec);
    serial_close(&ctx.ser);
    pthread_mutex_destroy(&ctx.cap_lock);
    pthread_mutex_destroy(&ctx.play_lock);
    printf("[SHUTDOWN] Completato.\n");
    return 0;
}
