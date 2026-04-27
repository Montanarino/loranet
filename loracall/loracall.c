/*
 * lora_voicecall.c  —  Chiamata vocale su DX-LR02 / DX-LR03
 *
 * Moduli testati: DX-Smart DX-LR02 / DX-LR03 (chip ASR6601)
 *
 * COME FUNZIONANO QUESTI MODULI (diverso da un modem GSM):
 * ─────────────────────────────────────────────────────────
 *  1. Si entra in modalità AT con "+++" + CR+LF → risponde "Entry AT"
 *  2. Si configura il modulo (frequenza, SF, BW …) con comandi AT
 *  3. Si esce con "AT+ENTM" → il modulo diventa un pipe seriale trasparente
 *  4. Tutto ciò che si scrive sulla seriale viene trasmesso via LoRa
 *     e ricevuto dall'altro modulo come bytes raw (nessun AT+SEND!)
 *
 * PROTOCOLLO APPLICATIVO (livello sopra la seriale trasparente):
 * ─────────────────────────────────────────────────────────────
 *  Frame: [0xAA][0x55][TYPE][LEN_HI][LEN_LO][PAYLOAD…][CRC8]
 *  Type:  0x01 CALL_REQ  0x02 CALL_ACK  0x03 CALL_END  0x10 AUDIO
 *
 * DIPENDENZE:
 *   apt install libopus-dev        (Linux)
 *   brew install opus              (macOS)
 *   + miniaudio.h  (header-only, Makefile lo scarica automaticamente)
 *
 * COMPILAZIONE:
 *   make
 *
 * UTILIZZO:
 *   ./lora_voicecall /dev/ttyUSB0 [baudrate] [freq_MHz]
 *   ./lora_voicecall /dev/ttyUSB0 9600 433.000
 *   ./lora_voicecall /dev/ttyUSB0 9600 868.500
 */

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <opus/opus.h>

/* ================================================================
 * CONFIGURAZIONE AUDIO
 * ================================================================ */
#define SAMPLE_RATE      8000
#define CHANNELS         1
/* 40 ms per frame = 320 campioni. Più lungo = meno overhead LoRa  */
#define FRAME_MS         40
#define FRAME_SAMPLES    (SAMPLE_RATE * FRAME_MS / 1000)   /* 320  */
#define OPUS_BITRATE     4800        /* bps — basso per LoRa        */
#define OPUS_MAX_PKT     128         /* byte max per pacchetto Opus */

/* ================================================================
 * PROTOCOLLO FRAMING
 * ================================================================ */
#define MAGIC_HI   0xAA
#define MAGIC_LO   0x55

#define PKT_CALL_REQ  0x01   /* richiesta chiamata  */
#define PKT_CALL_ACK  0x02   /* chiamata accettata  */
#define PKT_CALL_END  0x03   /* fine chiamata       */
#define PKT_AUDIO     0x10   /* frame audio Opus    */

#define FRAME_MAX  (6 + OPUS_MAX_PKT + 1)

/* ================================================================
 * STRUTTURE
 * ================================================================ */
typedef struct {
    int             fd;
    pthread_mutex_t lock;
} Serial;

typedef struct {
    Serial          ser;
    OpusEncoder    *enc;
    OpusDecoder    *dec;
    ma_device       cap_dev;
    ma_device       play_dev;

    int16_t         cap_pcm[FRAME_SAMPLES * 8];
    int             cap_n;
    pthread_mutex_t cap_lock;

    int16_t         play_pcm[FRAME_SAMPLES * 16];
    int             play_n;
    pthread_mutex_t play_lock;

    volatile bool   in_call;
    volatile bool   running;
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
 * SERIALE
 * ================================================================ */
static int serial_open(Serial *s, const char *dev, int baud)
{
    s->fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
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
    t.c_cc[VTIME] = 1;             /* 0.1 s read timeout */
    tcsetattr(s->fd, TCSANOW, &t);
    pthread_mutex_init(&s->lock, NULL);
    return 0;
}

static void serial_close(Serial *s)
{
    close(s->fd);
    pthread_mutex_destroy(&s->lock);
}

static int serial_write(Serial *s, const uint8_t *buf, int len)
{
    pthread_mutex_lock(&s->lock);
    int n = (int)write(s->fd, buf, len);
    pthread_mutex_unlock(&s->lock);
    return (n == len) ? 0 : -1;
}

static int serial_write_str(Serial *s, const char *str)
{
    return serial_write(s, (const uint8_t *)str, (int)strlen(str));
}

static int serial_read1(Serial *s, uint8_t *b)
{
    return (int)read(s->fd, b, 1);
}

/* Legge una riga (fino a \n) entro timeout_ms */
static int serial_readline(Serial *s, char *buf, int max, int ms_timeout)
{
    int n = 0, elapsed = 0;
    memset(buf, 0, max);
    while (elapsed < ms_timeout && n < max - 1) {
        uint8_t c;
        if (serial_read1(s, &c) == 1) {
            if (c == '\r') continue;
            if (c == '\n') { buf[n] = '\0'; return n; }
            buf[n++] = (char)c;
        } else {
            struct timespec ts = {0, 10000000L}; /* 10 ms */
            nanosleep(&ts, NULL);
            elapsed += 10;
        }
    }
    buf[n] = '\0';
    return n;
}

/* ================================================================
 * AT COMMANDS — DX-LR02 / DX-LR03
 *
 * Per entrare in modalità AT: invia "+++" + CRLF
 *   Il modulo risponde: "Entry AT"
 * Per uscire (modalità trasparente): "AT+ENTM" + CRLF
 *   Risponde: "OK" poi diventa pipe trasparente
 * ================================================================ */

static int at_enter(App *ctx)
{
    char resp[128];
    printf("[AT] Invio +++  (attesa 'Entry AT')…\n");
    serial_write_str(&ctx->ser, "+++\r\n");
    usleep(600000);  /* il modulo impiega ~500 ms a rispondere */
    int n = serial_readline(&ctx->ser, resp, sizeof(resp), 2000);
    printf("[AT] Risposta: '%s'\n", resp);
    if (n > 0 && (strstr(resp, "Entry AT") || strstr(resp, "+OK")
                  || strstr(resp, "OK")))
        return 0;
    /* secondo tentativo con semplice AT */
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
        if (strcmp(line, "OK") == 0 || strncmp(line, "+OK", 3) == 0)
            return 0;
        if (strncmp(line, "ERR", 3) == 0 || strncmp(line, "ERROR", 5) == 0)
            return -1;
    }
    return -1;
}

static int at_exit(App *ctx)
{
	at_cmd(ctx, "+++", NULL, 0);
	return 0;
}

/*
 * Configura il modulo per voice call.
 * SF7 + BW=500kHz → throughput ~21 kbps, sufficiente per Opus 4.8 kbps.
 */
static int lora_configure(App *ctx, float freq_mhz)
{
    char cmd[64], resp[128];

    at_cmd(ctx, "AT+LEVEL7", resp, sizeof(resp));         /* info fw    */
    at_cmd(ctx, "AT+MODE0", resp, sizeof(resp));       /* trasparente */

    printf("[LoRa] Configurato: %.3f MHz, SF7, BW500, CR4/5, 22dBm\n",
           freq_mhz);
    return 0;
}

/* ================================================================
 * FRAMING — costruisce e invia un frame
 * Layout: [0xAA][0x55][TYPE][LEN_HI][LEN_LO][PAYLOAD…][CRC8]
 * ================================================================ */
static int frame_send(App *ctx, uint8_t type,
                      const uint8_t *payload, uint16_t plen)
{
    uint8_t buf[FRAME_MAX];
    int     i = 0;
    buf[i++] = MAGIC_HI;
    buf[i++] = MAGIC_LO;
    buf[i++] = type;
    buf[i++] = (uint8_t)(plen >> 8);
    buf[i++] = (uint8_t)(plen & 0xFF);
    if (plen && payload) { memcpy(buf + i, payload, plen); i += plen; }
    buf[i] = crc8(buf, i);
    i++;
    return serial_write(&ctx->ser, buf, i);
}

/* ================================================================
 * FRAMING — parser a macchina a stati
 * ================================================================ */
typedef enum {
    ST_M1, ST_M2, ST_TYPE, ST_LH, ST_LL, ST_PAYLOAD, ST_CRC
} FState;

typedef struct {
    FState   st;
    uint8_t  type;
    uint16_t plen, pcnt;
    uint8_t  payload[OPUS_MAX_PKT + 16];
    uint8_t  raw[FRAME_MAX];
    int      raw_n;
} FParser;

static void fp_reset(FParser *fp)
{
    fp->st = ST_M1; fp->raw_n = 0; fp->pcnt = 0;
}

/* Ritorna true se un frame completo e valido è disponibile in fp */
static bool fp_feed(FParser *fp, uint8_t b)
{
    switch (fp->st) {
    case ST_M1:
        if (b == MAGIC_HI) {
            fp->raw_n = 0;
            fp->raw[fp->raw_n++] = b;
            fp->st = ST_M2;
        }
        break;
    case ST_M2:
        fp->raw[fp->raw_n++] = b;
        fp->st = (b == MAGIC_LO) ? ST_TYPE : ST_M1;
        break;
    case ST_TYPE:
        fp->type = b; fp->raw[fp->raw_n++] = b; fp->st = ST_LH;
        break;
    case ST_LH:
        fp->plen = (uint16_t)(b << 8); fp->raw[fp->raw_n++] = b;
        fp->st = ST_LL;
        break;
    case ST_LL:
        fp->plen |= b; fp->raw[fp->raw_n++] = b; fp->pcnt = 0;
        if (fp->plen == 0) { fp->st = ST_CRC; break; }
        if (fp->plen > sizeof(fp->payload)) { fp_reset(fp); break; }
        fp->st = ST_PAYLOAD;
        break;
    case ST_PAYLOAD:
        fp->payload[fp->pcnt++] = b;
        fp->raw[fp->raw_n++]    = b;
        if (fp->pcnt == fp->plen) fp->st = ST_CRC;
        break;
    case ST_CRC: {
        uint8_t exp = crc8(fp->raw, fp->raw_n);
        fp_reset(fp);
        return (b == exp);
    }
    }
    return false;
}

/* ================================================================
 * AUDIO CALLBACKS
 * ================================================================ */
static void cap_cb(ma_device *d, void *out,
                   const void *in, ma_uint32 n)
{
    (void)out;
    App *ctx = (App *)d->pUserData;
    if (!ctx->in_call) return;
    const int16_t *src = (const int16_t *)in;
    pthread_mutex_lock(&ctx->cap_lock);
    int avail = (int)(sizeof(ctx->cap_pcm)/2) - ctx->cap_n;
    int copy  = ((int)n < avail) ? (int)n : avail;
    memcpy(ctx->cap_pcm + ctx->cap_n, src, copy * 2);
    ctx->cap_n += copy;
    pthread_mutex_unlock(&ctx->cap_lock);
}

static void play_cb(ma_device *d, void *out,
                    const void *in, ma_uint32 n)
{
    (void)in;
    App     *ctx = (App *)d->pUserData;
    int16_t *dst = (int16_t *)out;
    pthread_mutex_lock(&ctx->play_lock);
    int copy = ((int)n < ctx->play_n) ? (int)n : ctx->play_n;
    if (copy > 0) {
        memcpy(dst, ctx->play_pcm, copy * 2);
        memmove(ctx->play_pcm, ctx->play_pcm + copy,
                (ctx->play_n - copy) * 2);
        ctx->play_n -= copy;
    }
    if (copy < (int)n)
        memset(dst + copy, 0, ((int)n - copy) * 2);
    pthread_mutex_unlock(&ctx->play_lock);
}

/* ================================================================
 * THREAD TX  microfono → Opus → frame LoRa
 * ================================================================ */
static void *tx_thread(void *arg)
{
    App     *ctx = (App *)arg;
    int16_t  pcm[FRAME_SAMPLES];
    uint8_t  pkt[OPUS_MAX_PKT];

    printf("[TX] Thread avviato.\n");
    while (ctx->running) {
        if (!ctx->in_call) { usleep(10000); continue; }

        /* Attendi un frame completo dal microfono */
        bool got = false;
        for (int w = 0; w < 250 && !got; w++) {
            pthread_mutex_lock(&ctx->cap_lock);
            if (ctx->cap_n >= FRAME_SAMPLES) {
                memcpy(pcm, ctx->cap_pcm, FRAME_SAMPLES * 2);
                memmove(ctx->cap_pcm, ctx->cap_pcm + FRAME_SAMPLES,
                        (ctx->cap_n - FRAME_SAMPLES) * 2);
                ctx->cap_n -= FRAME_SAMPLES;
                got = true;
            }
            pthread_mutex_unlock(&ctx->cap_lock);
            if (!got) usleep(2000);
        }
        if (!got) continue;

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
 * ================================================================ */
static void *rx_thread(void *arg)
{
    App     *ctx = (App *)arg;
    FParser  fp;
    int16_t  pcm[FRAME_SAMPLES * 2];
    fp_reset(&fp);

    printf("[RX] Thread avviato.\n");
    while (ctx->running) {
        uint8_t b;
        if (serial_read1(&ctx->ser, &b) != 1) { usleep(1000); continue; }
        if (!fp_feed(&fp, b)) continue;

        switch (fp.type) {

        case PKT_CALL_REQ:
            printf("\n[CALL] *** Chiamata in arrivo! Premi 'a' per rispondere ***\n> ");
            fflush(stdout);
            break;

        case PKT_CALL_ACK:
            if (!ctx->in_call) {
                ctx->in_call = true;
                printf("\n[CALL] Chiamata accettata — connesso!\n> ");
                fflush(stdout);
            }
            break;

        case PKT_CALL_END:
            ctx->in_call = false;
            printf("\n[CALL] Chiamata terminata dall'altro capo.\n> ");
            fflush(stdout);
            break;

        case PKT_AUDIO:
            if (!ctx->in_call) break;
            {
                int s = opus_decode(ctx->dec, fp.payload, fp.plen,
                                    pcm, FRAME_SAMPLES, 0);
                if (s < 0) {
                    fprintf(stderr, "[RX] Opus: %s\n", opus_strerror(s));
                    break;
                }
                pthread_mutex_lock(&ctx->play_lock);
                int space = (int)(sizeof(ctx->play_pcm)/2) - ctx->play_n;
                int copy  = (s < space) ? s : space;
                memcpy(ctx->play_pcm + ctx->play_n, pcm, copy * 2);
                ctx->play_n += copy;
                pthread_mutex_unlock(&ctx->play_lock);
            }
            break;
        }
    }
    printf("[RX] Thread terminato.\n");
    return NULL;
}

/* ================================================================
 * MAIN
 * ================================================================ */
static void sig_handler(int s)
{
    (void)s;
    if (g_app) { g_app->running = false; g_app->in_call = false; }
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr,
            "Uso: %s <porta> [baud] [freq_MHz]\n"
            "Es:  %s /dev/ttyUSB0 9600 433.000\n"
            "Es:  %s /dev/ttyUSB0 9600 868.500\n",
            argv[0], argv[0], argv[0]);
        return 1;
    }

    const char *dev  = argv[1];
    int         baud = (argc >= 3) ? atoi(argv[2]) : 9600;
    float       freq = (argc >= 4) ? (float)atof(argv[3]) : 433.000f;

    static App ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.running = true;
    g_app = &ctx;
    pthread_mutex_init(&ctx.cap_lock,  NULL);
    pthread_mutex_init(&ctx.play_lock, NULL);

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* 1. Seriale */
    printf("[INIT] Apertura %s @ %d baud…\n", dev, baud);
    if (serial_open(&ctx.ser, dev, baud) != 0) return 1;

    /* 2. Modulo LoRa */
    printf("[INIT] Configurazione DX-LR02/LR03…\n");
    if (at_enter(&ctx) != 0) {
        fprintf(stderr,
            "\n[ERR] Modulo non risponde!\n"
            "  • Verifica che l'antenna sia collegata\n"
            "  • Verifica baud rate (default: 9600)\n"
            "  • Prova a scollegare e ricollegare il cavo USB\n"
            "  • Verifica connessioni: RXD→TX, TXD→RX, VCC→5V, GND→GND\n");
        serial_close(&ctx.ser);
        return 1;
    }
    if (lora_configure(&ctx, freq) != 0 || at_exit(&ctx) != 0) {
        serial_close(&ctx.ser);
        return 1;
    }
    printf("[INIT] Modulo in modalità trasparente.\n");

    /* 3. Opus */
    int err;
    ctx.enc = opus_encoder_create(SAMPLE_RATE, CHANNELS,
                                  OPUS_APPLICATION_VOIP, &err);
    if (err != OPUS_OK) {
        fprintf(stderr, "opus_encoder_create: %s\n", opus_strerror(err));
        goto cleanup_serial;
    }
    opus_encoder_ctl(ctx.enc, OPUS_SET_BITRATE(OPUS_BITRATE));
    opus_encoder_ctl(ctx.enc, OPUS_SET_COMPLEXITY(2));
    opus_encoder_ctl(ctx.enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    opus_encoder_ctl(ctx.enc, OPUS_SET_DTX(1)); /* soppressione silenzi */

    ctx.dec = opus_decoder_create(SAMPLE_RATE, CHANNELS, &err);
    if (err != OPUS_OK) {
        fprintf(stderr, "opus_decoder_create: %s\n", opus_strerror(err));
        goto cleanup_opus;
    }

    /* 4. Audio (miniaudio) */
    {
        ma_device_config c = ma_device_config_init(ma_device_type_capture);
        c.capture.format   = ma_format_s16;
        c.capture.channels = CHANNELS;
        c.sampleRate       = SAMPLE_RATE;
        c.dataCallback     = cap_cb;
        c.pUserData        = &ctx;
        if (ma_device_init(NULL, &c, &ctx.cap_dev) != MA_SUCCESS) {
            fprintf(stderr, "[ERR] Init microfono\n");
            goto cleanup_opus;
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
            fprintf(stderr, "[ERR] Init altoparlante\n");
            goto cleanup_audio_cap;
        }
    }
    ma_device_start(&ctx.cap_dev);
    ma_device_start(&ctx.play_dev);
    printf("[INIT] Audio OK (8 kHz, mono, Opus %d bps, frame %d ms)\n",
           OPUS_BITRATE, FRAME_MS);

    /* 5. Thread TX / RX */
    pthread_create(&ctx.tx_tid, NULL, tx_thread, &ctx);
    pthread_create(&ctx.rx_tid, NULL, rx_thread, &ctx);

    /* 6. Menu */
    printf("\n");
    printf("╔═══════════════════════════════════════╗\n");
    printf("║    LoRa Voice Call — DX-LR02 / LR03   ║\n");
    printf("║  Frequenza: %7.3f MHz | SF7 BW500   ║\n", freq);
    printf("╠═══════════════════════════════════════╣\n");
    printf("║  c  → Chiama (invia segnale)          ║\n");
    printf("║  a  → Rispondi alla chiamata           ║\n");
    printf("║  h  → Riaggancia                      ║\n");
    printf("║  s  → Mostra stato                    ║\n");
    printf("║  q  → Esci                            ║\n");
    printf("╚═══════════════════════════════════════╝\n");
    printf("> ");
    fflush(stdout);

    char line[32];
    while (ctx.running && fgets(line, sizeof(line), stdin)) {
        line[strcspn(line, "\r\n")] = '\0';
        switch (line[0]) {
        case 'q':
            ctx.running = false;
            break;
        case 'c':
            frame_send(&ctx, PKT_CALL_REQ, NULL, 0);
            printf("[CALL] Segnale inviato — attendo risposta…\n");
            break;
        case 'a':
            frame_send(&ctx, PKT_CALL_ACK, NULL, 0);
            ctx.in_call = true;
            printf("[CALL] Risposto — connesso!\n");
            break;
        case 'h':
            ctx.in_call = false;
            frame_send(&ctx, PKT_CALL_END, NULL, 0);
            printf("[CALL] Riagganciato.\n");
            break;
        case 's':
            printf("[STATO] %s | %.3f MHz | Opus %d bps | SF7 BW500\n",
                   ctx.in_call ? "IN CHIAMATA 📞" : "LIBERO",
                   freq, OPUS_BITRATE);
            break;
        default:
            if (line[0] != '\0')
                printf("[?] Comando sconosciuto. Usa: c a h s q\n");
        }
        if (ctx.running) { printf("> "); fflush(stdout); }
    }

    /* Cleanup */
    ctx.running = false;
    ctx.in_call = false;
    frame_send(&ctx, PKT_CALL_END, NULL, 0);

    pthread_join(ctx.tx_tid, NULL);
    pthread_join(ctx.rx_tid, NULL);

    ma_device_stop(&ctx.play_dev);
    ma_device_stop(&ctx.cap_dev);
    ma_device_uninit(&ctx.play_dev);
cleanup_audio_cap:
    ma_device_uninit(&ctx.cap_dev);
cleanup_opus:
    if (ctx.enc) opus_encoder_destroy(ctx.enc);
    if (ctx.dec) opus_decoder_destroy(ctx.dec);
cleanup_serial:
    serial_close(&ctx.ser);
    pthread_mutex_destroy(&ctx.cap_lock);
    pthread_mutex_destroy(&ctx.play_lock);
    printf("[SHUTDOWN] Completato.\n");
    return 0;
}
