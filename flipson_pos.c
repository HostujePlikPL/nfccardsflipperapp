/**
 * FLIPSON POS Reader & Card Writer — Zaawansowana Aplikacja Flipper Zero
 * Bezpośredni odczyt kart zbliżeniowych EMV (PAN 16 cyfr, data, holder)
 * ORAZ zapis i odczyt pełnych danych kart bankowych na czystych kartach NFC (MIFARE Classic / Ultralight / NTAG)
 * Bezpieczna transmisja pakietów JSON przez port szeregowy USB VCP oraz UART GPIO (Pin 13 TX).
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_serial.h>
#include <furi_hal_serial_control.h>
#include <furi_hal_serial_types.h>
#include <furi/core/thread.h>
#include <furi_hal_speaker.h>
#include <furi_hal_light.h>
#include <furi_hal_vibro.h>
#include <furi_hal_rtc.h>
#include <gui/gui.h>
#include <gui/view_port.h>
#include <gui/canvas.h>
#include <input/input.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <nfc/nfc.h>
#include <nfc/nfc_poller.h>
#include <nfc/protocols/iso14443_3a/iso14443_3a.h>
#include <nfc/protocols/iso14443_3a/iso14443_3a_poller_sync.h>
#include <nfc/protocols/iso14443_4a/iso14443_4a.h>
#include <nfc/protocols/iso14443_4a/iso14443_4a_poller.h>
#include <nfc/protocols/mf_classic/mf_classic.h>
#include <nfc/protocols/mf_classic/mf_classic_poller_sync.h>
#include <nfc/protocols/mf_ultralight/mf_ultralight.h>
#include <nfc/protocols/mf_ultralight/mf_ultralight_poller_sync.h>
#include <toolbox/bit_buffer.h>
#include <ctype.h>

#include "flipson_emv.h"
#include <furi_hal_usb_cdc.h>
#include <toolbox/cli/cli_registry.h>
#define RECORD_CLI "cli"

#define TAG "FlipsonPos"
#define NFC_THREAD_STACK_SIZE (8 * 1024)

typedef enum {
    FlipsonStateWaiting,        // Oczekiwanie na kartę w trybie czytnika POS
    FlipsonStateReading,        // Czytanie APDU / sektorów
    FlipsonStateSuccess,        // Sukces odczytu karty, wysłano do POS
    FlipsonStateWriteWaiting,   // Oczekiwanie na czystą kartę do zapisu danych
    FlipsonStateWriteSuccess,   // Sukces programowania czystej karty
    FlipsonStateError
} FlipsonState;

typedef struct {
    FuriMutex* mutex;
    FlipsonState state;
    FlipsonCardData current_card;
    FlipsonCardWriteRequest write_req;
    uint32_t tap_count;
    char last_uid[32];
    uint32_t last_tap_tick;
    char status_msg[64];
    FuriHalSerialHandle* serial_handle;
    Nfc* nfc;
    FuriThread* nfc_worker_thread;
    volatile bool worker_running;
    volatile bool running;
    char rx_line_buf[256];
    size_t rx_line_len;
    // Duże bufory zaalokowane w stercie (heap), aby chronić stos wątków przed przepełnieniem (MPU Fault)
    char json_tx_buf[512];
    uint8_t apdu_rx_buf[256];
    Iso14443_3aData data3a;
    Iso14443_3aData dummy3a;
} FlipsonAppContext;

// Sygnał dźwiękowy autoryzacji zbliżenia (charakterystyczny dwuton z terminali płatniczych)
static void flipson_play_pos_chime(void) {
    if (furi_hal_speaker_is_mine() || furi_hal_speaker_acquire(100)) {
        furi_hal_speaker_start(1760.0f, 0.4f);
        furi_delay_ms(70);
        furi_hal_speaker_start(2640.0f, 0.5f);
        furi_delay_ms(120);
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
    }
    furi_hal_light_set(LightGreen, 255);
    furi_hal_vibro_on(true);
    furi_delay_ms(40);
    furi_hal_vibro_on(false);
    furi_delay_ms(150);
    furi_hal_light_set(LightGreen, 0);
}

// Potrójny radosny sygnał zaprogramowania nowej karty NFC
static void flipson_play_write_success_chime(void) {
    if (furi_hal_speaker_is_mine() || furi_hal_speaker_acquire(100)) {
        for (int i = 0; i < 3; i++) {
            furi_hal_speaker_start(2000.0f + (i * 400.0f), 0.4f);
            furi_delay_ms(50);
            furi_hal_speaker_stop();
            furi_delay_ms(30);
        }
        furi_hal_speaker_release();
    }
    furi_hal_light_set(LightBlue, 255);
    furi_hal_vibro_on(true);
    furi_delay_ms(60);
    furi_hal_vibro_on(false);
    furi_delay_ms(150);
    furi_hal_light_set(LightBlue, 0);
}

// Niski sygnał błędu
static void flipson_play_error_buzz(void) {
    if (furi_hal_speaker_is_mine() || furi_hal_speaker_acquire(100)) {
        furi_hal_speaker_start(220.0f, 0.3f);
        furi_delay_ms(150);
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
    }
    furi_hal_light_set(LightRed, 255);
    furi_delay_ms(100);
    furi_hal_light_set(LightRed, 0);
}

// Bezpieczna transmisja pakietu JSON przez USB VCP stdout oraz UART GPIO
static void flipson_transmit_json_packet(FlipsonAppContext* app, const char* json_buffer) {
    if (!app || !json_buffer) return;
    size_t len = strlen(json_buffer);

    // 1. Bezpośredni buforowany zapis do strumienia wyjściowego wątku (VCP / CLI)
    furi_thread_stdout_write(json_buffer, len);
    furi_thread_stdout_flush();

    // 2. Bezpośrednia transmisja przez USB VCP (jeśli DTR aktywny)
    if (furi_hal_cdc_get_ctrl_line_state(0) & CdcCtrlLineDTR) {
        furi_hal_cdc_send(0, (uint8_t*)json_buffer, len);
    }

    // 3. Transmisja do systemu logów Furi (przechwytywana przez mostek)
    FURI_LOG_I(TAG, "JSON_TAP: %s", json_buffer);

    // 4. Sprzętowy UART na pinie 13 GPIO (115200 baud)
    if (app->serial_handle) {
        furi_hal_serial_tx(app->serial_handle, (const uint8_t*)json_buffer, len);
        furi_hal_serial_tx_wait_complete(app->serial_handle);
    }
}

// Wysłanie odczytanej karty (CARD_TAP)
static void flipson_transmit_card_json(FlipsonAppContext* app, const FlipsonCardData* card) {
    if (!app || !card) return;
    uint32_t now_ts = furi_hal_rtc_get_timestamp();

    snprintf(
        app->json_tx_buf,
        sizeof(app->json_tx_buf),
        "{\"event\":\"CARD_TAP\",\"source\":\"FLIPPER_ZERO\",\"device\":\"FLIPPER_POS_1.3\","
        "\"protocol\":\"%s\",\"uid\":\"%s\",\"card_number\":\"%s\",\"exp_month\":\"%s\","
        "\"exp_year\":\"%s\",\"cvc\":\"%s\",\"cardholder\":\"%s\",\"token\":\"%s\",\"ts\":%lu}\r\n",
        card_protocol_name(card->protocol),
        card->uid_hex,
        card->has_pan ? card->pan : "",
        card->exp_month,
        card->exp_year,
        card->cvc,
        card->cardholder,
        card->secure_token,
        (unsigned long)now_ts
    );

    flipson_transmit_json_packet(app, app->json_tx_buf);
}

// Wysłanie potwierdzenia zapisu (CARD_WRITTEN)
static void flipson_transmit_write_ack(FlipsonAppContext* app, const FlipsonCardData* card, bool success) {
    if (!app || !card) return;
    uint32_t now_ts = furi_hal_rtc_get_timestamp();
    snprintf(
        app->json_tx_buf,
        sizeof(app->json_tx_buf),
        "{\"event\":\"CARD_WRITTEN\",\"status\":\"%s\",\"uid\":\"%s\",\"protocol\":\"%s\",\"ts\":%lu}\r\n",
        success ? "OK" : "ERROR",
        card->uid_hex,
        card_protocol_name(card->protocol),
        (unsigned long)now_ts
    );

    flipson_transmit_json_packet(app, app->json_tx_buf);
}

// ── OBSŁUGA ISO14443-4 APDU (EMV) ─────────────────────────────────────────────
static const uint8_t APDU_SELECT_PPSE[] = {
    0x00, 0xA4, 0x04, 0x00, 0x0E,
    '2', 'P', 'A', 'Y', '.', 'S', 'Y', 'S', '.', 'D', 'D', 'F', '0', '1',
    0x00
};

static const uint8_t APDU_SELECT_VISA[] = {
    0x00, 0xA4, 0x04, 0x00, 0x07,
    0xA0, 0x00, 0x00, 0x00, 0x03, 0x10, 0x10,
    0x00
};

static const uint8_t APDU_SELECT_MC[] = {
    0x00, 0xA4, 0x04, 0x00, 0x07,
    0xA0, 0x00, 0x00, 0x00, 0x04, 0x10, 0x10,
    0x00
};

static const uint8_t APDU_READ_RECORD_1[] = {
    0x00, 0xB2, 0x01, 0x0C, 0x00
};

static const uint8_t APDU_READ_RECORD_2[] = {
    0x00, 0xB2, 0x01, 0x14, 0x00
};

static bool flipson_apdu_exchange(
    Iso14443_4aPoller* iso_poller,
    const uint8_t* cmd,
    size_t cmd_len,
    uint8_t* out_data,
    size_t out_max,
    size_t* out_len) {

    if (!iso_poller || !cmd || cmd_len == 0 || !out_data || !out_len) return false;

    BitBuffer* tx_buf = bit_buffer_alloc(cmd_len + 8);
    BitBuffer* rx_buf = bit_buffer_alloc(out_max + 8);

    bit_buffer_copy_bytes(tx_buf, cmd, cmd_len);

    Iso14443_4aError err = iso14443_4a_poller_send_block(iso_poller, tx_buf, rx_buf);

    bool success = false;
    if (err == Iso14443_4aErrorNone) {
        size_t rx_bytes = bit_buffer_get_size_bytes(rx_buf);
        if (rx_bytes > 0 && rx_bytes <= out_max) {
            bit_buffer_write_bytes(rx_buf, out_data, rx_bytes);
            *out_len = rx_bytes;
            success = true;
        }
    }

    bit_buffer_free(tx_buf);
    bit_buffer_free(rx_buf);
    return success;
}

static void flipson_emv_read_records(Iso14443_4aPoller* iso_poller, FlipsonCardData* card, uint8_t* rx_buf, size_t rx_max) {
    size_t rx_len = 0;

    if (flipson_apdu_exchange(iso_poller, APDU_READ_RECORD_1, sizeof(APDU_READ_RECORD_1), rx_buf, rx_max, &rx_len)) {
        if (rx_len > 2) {
            emv_parse_record_tlv(rx_buf, rx_len - 2, card);
        }
    }

    if (!card->has_pan) {
        if (flipson_apdu_exchange(iso_poller, APDU_READ_RECORD_2, sizeof(APDU_READ_RECORD_2), rx_buf, rx_max, &rx_len)) {
            if (rx_len > 2) {
                emv_parse_record_tlv(rx_buf, rx_len - 2, card);
            }
        }
    }
}

static void flipson_emv_perform_full_read(Iso14443_4aPoller* iso_poller, FlipsonCardData* card, uint8_t* rx_buf, size_t rx_max) {
    size_t rx_len = 0;

    // 1. SELECT PPSE
    if (flipson_apdu_exchange(iso_poller, APDU_SELECT_PPSE, sizeof(APDU_SELECT_PPSE), rx_buf, rx_max, &rx_len)) {
        if (rx_len > 2) emv_parse_record_tlv(rx_buf, rx_len - 2, card);
    }

    // 2. SELECT VISA
    if (!card->has_pan) {
        if (flipson_apdu_exchange(iso_poller, APDU_SELECT_VISA, sizeof(APDU_SELECT_VISA), rx_buf, rx_max, &rx_len)) {
            card->protocol = CARD_PROTO_EMV_VISA;
            flipson_emv_read_records(iso_poller, card, rx_buf, rx_max);
        }
    }

    // 3. SELECT MASTERCARD
    if (!card->has_pan) {
        if (flipson_apdu_exchange(iso_poller, APDU_SELECT_MC, sizeof(APDU_SELECT_MC), rx_buf, rx_max, &rx_len)) {
            card->protocol = CARD_PROTO_EMV_MASTERCARD;
            flipson_emv_read_records(iso_poller, card, rx_buf, rx_max);
        }
    }
}

typedef struct {
    FlipsonAppContext* app;
    FlipsonCardData* card;
    volatile bool completed;
} EmvSessionContext;

static NfcCommand flipson_emv_poller_cb(NfcGenericEvent event, void* context) {
    EmvSessionContext* ctx = (EmvSessionContext*)context;
    if (event.protocol == NfcProtocolIso14443_4a) {
        Iso14443_4aPollerEvent* iso_event = (Iso14443_4aPollerEvent*)event.event_data;
        if (iso_event && iso_event->type == Iso14443_4aPollerEventTypeReady) {
            Iso14443_4aPoller* iso_poller = (Iso14443_4aPoller*)event.instance;
            flipson_emv_perform_full_read(iso_poller, ctx->card, ctx->app->apdu_rx_buf, sizeof(ctx->app->apdu_rx_buf));
            ctx->completed = true;
            return NfcCommandStop;
        } else if (iso_event && iso_event->type == Iso14443_4aPollerEventTypeError) {
            ctx->completed = true;
            return NfcCommandStop;
        }
    }
    return NfcCommandContinue;
}

static bool flipson_read_emv_4a(FlipsonAppContext* app, FlipsonCardData* card) {
    if (!app || !app->nfc || !card) return false;

    NfcPoller* poller = nfc_poller_alloc(app->nfc, NfcProtocolIso14443_4a);
    if (!poller) return false;

    EmvSessionContext ctx = { .app = app, .card = card, .completed = false };
    nfc_poller_start(poller, flipson_emv_poller_cb, &ctx);

    uint32_t start_tick = furi_get_tick();
    while (!ctx.completed && (furi_get_tick() - start_tick) < furi_ms_to_ticks(600)) {
        furi_delay_ms(15);
    }

    nfc_poller_stop(poller);
    nfc_poller_free(poller);
    furi_delay_ms(25);
    return card->has_pan;
}

// ── FABRYCZNE KLUCZE MIFARE CLASSIC ──────────────────────────────────────────
static const MfClassicKey DEFAULT_KEYS[] = {
    { .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
    { .data = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5} },
    { .data = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7} },
    { .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
    { .data = {0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5} },
    { .data = {0x4D, 0x3A, 0x99, 0xC3, 0x51, 0xDD} },
    { .data = {0x1A, 0x98, 0x2C, 0x7E, 0x45, 0x9A} },
    { .data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF} }
};

// ── ZAPIS I ODCZYT MIFARE CLASSIC (Sektor 1, Bloki 4, 5, 6) ───────────────────
static bool flipson_mifare_write_data(Nfc* nfc, const FlipsonCardWriteRequest* req) {
    if (!nfc || !req) return false;

    bool authed = false;
    MfClassicKey active_key;
    MfClassicKeyType active_key_type = MfClassicKeyTypeA;

    for (size_t k = 0; k < COUNT_OF(DEFAULT_KEYS); k++) {
        active_key = DEFAULT_KEYS[k];
        MfClassicBlock dummy;
        if (mf_classic_poller_sync_read_block(nfc, 4, &active_key, MfClassicKeyTypeA, &dummy) == MfClassicErrorNone) {
            authed = true;
            active_key_type = MfClassicKeyTypeA;
            break;
        }
        if (mf_classic_poller_sync_read_block(nfc, 4, &active_key, MfClassicKeyTypeB, &dummy) == MfClassicErrorNone) {
            authed = true;
            active_key_type = MfClassicKeyTypeB;
            break;
        }
    }

    if (!authed) return false;

    // Blok 4: Numer karty (16 cyfr PAN)
    MfClassicBlock b4;
    memset(b4.data, ' ', sizeof(b4.data));
    size_t pan_len = strlen(req->pan);
    if (pan_len > 16) pan_len = 16;
    memcpy(b4.data, req->pan, pan_len);

    if (mf_classic_poller_sync_write_block(nfc, 4, &active_key, active_key_type, &b4) != MfClassicErrorNone) {
        return false;
    }

    // Blok 5: Data ważności (MMYY 4 znaki) + CVC (3 znaki) + Właściciel (9 znaków)
    MfClassicBlock b5;
    memset(b5.data, ' ', sizeof(b5.data));
    char tmp5[32];
    snprintf(tmp5, sizeof(tmp5), "%-4.4s%-3.3s%-9.9s", req->exp, req->cvc, req->cardholder);
    memcpy(b5.data, tmp5, 16);

    if (mf_classic_poller_sync_write_block(nfc, 5, &active_key, active_key_type, &b5) != MfClassicErrorNone) {
        return false;
    }

    // Blok 6: Bezpieczny token bankowy (16 znaków)
    MfClassicBlock b6;
    memset(b6.data, ' ', sizeof(b6.data));
    char tmp6[32];
    snprintf(tmp6, sizeof(tmp6), "%-16.16s", req->secure_token);
    memcpy(b6.data, tmp6, 16);

    if (mf_classic_poller_sync_write_block(nfc, 6, &active_key, active_key_type, &b6) != MfClassicErrorNone) {
        return false;
    }

    return true;
}

static bool flipson_mifare_read_custom_data(Nfc* nfc, FlipsonCardData* card) {
    if (!nfc || !card) return false;

    MfClassicKey active_key;
    MfClassicKeyType active_key_type = MfClassicKeyTypeA;

    for (size_t k = 0; k < COUNT_OF(DEFAULT_KEYS); k++) {
        active_key = DEFAULT_KEYS[k];
        MfClassicBlock b4;
        bool authed = false;
        if (mf_classic_poller_sync_read_block(nfc, 4, &active_key, MfClassicKeyTypeA, &b4) == MfClassicErrorNone) {
            authed = true;
            active_key_type = MfClassicKeyTypeA;
        } else if (mf_classic_poller_sync_read_block(nfc, 4, &active_key, MfClassicKeyTypeB, &b4) == MfClassicErrorNone) {
            authed = true;
            active_key_type = MfClassicKeyTypeB;
        }

        if (authed) {
            if (isdigit(b4.data[0]) && isdigit(b4.data[1])) {
                size_t pan_idx = 0;
                while (pan_idx < 16 && isdigit(b4.data[pan_idx])) {
                    card->pan[pan_idx] = b4.data[pan_idx];
                    pan_idx++;
                }
                card->pan[pan_idx] = '\0';
                if (pan_idx >= 12) {
                    card->has_pan = true;
                    card->protocol = CARD_PROTO_MIFARE_SECURE;
                }

                // Odczytaj Blok 5 (Exp, CVC, Holder)
                MfClassicBlock b5;
                if (mf_classic_poller_sync_read_block(nfc, 5, &active_key, active_key_type, &b5) == MfClassicErrorNone) {
                    if (isdigit(b5.data[0]) && isdigit(b5.data[1])) {
                        card->exp_month[0] = b5.data[0];
                        card->exp_month[1] = b5.data[1];
                        card->exp_month[2] = '\0';
                    }
                    if (isdigit(b5.data[2]) && isdigit(b5.data[3])) {
                        snprintf(card->exp_year, sizeof(card->exp_year), "20%c%c", b5.data[2], b5.data[3]);
                    }
                    if (isdigit(b5.data[4]) && isdigit(b5.data[5]) && isdigit(b5.data[6])) {
                        card->cvc[0] = b5.data[4];
                        card->cvc[1] = b5.data[5];
                        card->cvc[2] = b5.data[6];
                        card->cvc[3] = '\0';
                    }
                    char raw_name[10];
                    memcpy(raw_name, &b5.data[7], 9);
                    raw_name[9] = '\0';
                    size_t nlen = strlen(raw_name);
                    while (nlen > 0 && raw_name[nlen - 1] == ' ') raw_name[--nlen] = '\0';
                    if (nlen > 0) {
                        strncpy(card->cardholder, raw_name, sizeof(card->cardholder) - 1);
                    }
                }

                // Odczytaj Blok 6 (Token)
                MfClassicBlock b6;
                if (mf_classic_poller_sync_read_block(nfc, 6, &active_key, active_key_type, &b6) == MfClassicErrorNone) {
                    char raw_tkn[17];
                    memcpy(raw_tkn, b6.data, 16);
                    raw_tkn[16] = '\0';
                    size_t tlen = strlen(raw_tkn);
                    while (tlen > 0 && raw_tkn[tlen - 1] == ' ') raw_tkn[--tlen] = '\0';
                    if (tlen > 0) {
                        strncpy(card->secure_token, raw_tkn, sizeof(card->secure_token) - 1);
                        card->has_token = true;
                    }
                }
                return true;
            }
        }
    }
    return false;
}

// ── ZAPIS I ODCZYT MIFARE ULTRALIGHT / NTAG (Strony 4 do 17) ───────────────────
static bool flipson_mful_write_data(Nfc* nfc, const FlipsonCardWriteRequest* req) {
    if (!nfc || !req) return false;

    // Pages 4..7: PAN 16 cyfr (4 bajty na stronę)
    char pan16[17] = "                ";
    size_t plen = strlen(req->pan);
    if (plen > 16) plen = 16;
    memcpy(pan16, req->pan, plen);

    for (uint16_t p = 0; p < 4; p++) {
        MfUltralightPage page;
        memcpy(page.data, &pan16[p * 4], 4);
        if (mf_ultralight_poller_sync_write_page(nfc, 4 + p, &page) != MfUltralightErrorNone) {
            return false;
        }
    }

    // Page 8: Exp (MMYY, 4 bajty)
    char exp4[5] = "    ";
    size_t elen = strlen(req->exp);
    if (elen > 4) elen = 4;
    memcpy(exp4, req->exp, elen);
    MfUltralightPage p8;
    memcpy(p8.data, exp4, 4);
    if (mf_ultralight_poller_sync_write_page(nfc, 8, &p8) != MfUltralightErrorNone) {
        return false;
    }

    // Page 9: CVC (3 cyfry + spacja)
    char cvc4[5] = "    ";
    size_t clen = strlen(req->cvc);
    if (clen > 3) clen = 3;
    memcpy(cvc4, req->cvc, clen);
    MfUltralightPage p9;
    memcpy(p9.data, cvc4, 4);
    if (mf_ultralight_poller_sync_write_page(nfc, 9, &p9) != MfUltralightErrorNone) {
        return false;
    }

    // Pages 10..13: Holder (16 bajtów)
    char holder16[17] = "                ";
    size_t hlen = strlen(req->cardholder);
    if (hlen > 16) hlen = 16;
    memcpy(holder16, req->cardholder, hlen);
    for (uint16_t p = 0; p < 4; p++) {
        MfUltralightPage page;
        memcpy(page.data, &holder16[p * 4], 4);
        if (mf_ultralight_poller_sync_write_page(nfc, 10 + p, &page) != MfUltralightErrorNone) {
            return false;
        }
    }

    // Pages 14..17: Token (16 bajtów)
    char token16[17] = "                ";
    size_t tlen = strlen(req->secure_token);
    if (tlen > 16) tlen = 16;
    memcpy(token16, req->secure_token, tlen);
    for (uint16_t p = 0; p < 4; p++) {
        MfUltralightPage page;
        memcpy(page.data, &token16[p * 4], 4);
        if (mf_ultralight_poller_sync_write_page(nfc, 14 + p, &page) != MfUltralightErrorNone) {
            return false;
        }
    }

    return true;
}

static bool flipson_mful_read_custom_data(Nfc* nfc, FlipsonCardData* card) {
    if (!nfc || !card) return false;

    MfUltralightPage p4;
    if (mf_ultralight_poller_sync_read_page(nfc, 4, &p4) != MfUltralightErrorNone) {
        return false;
    }

    if (!isdigit(p4.data[0]) || !isdigit(p4.data[1])) {
        return false;
    }

    // Odczytaj 16 cyfr PAN (Pages 4..7)
    char raw_pan[17];
    memcpy(&raw_pan[0], p4.data, 4);

    for (uint16_t p = 1; p < 4; p++) {
        MfUltralightPage page;
        if (mf_ultralight_poller_sync_read_page(nfc, 4 + p, &page) == MfUltralightErrorNone) {
            memcpy(&raw_pan[p * 4], page.data, 4);
        } else {
            return false;
        }
    }
    raw_pan[16] = '\0';

    size_t pan_idx = 0;
    while (pan_idx < 16 && isdigit((unsigned char)raw_pan[pan_idx])) {
        card->pan[pan_idx] = raw_pan[pan_idx];
        pan_idx++;
    }
    card->pan[pan_idx] = '\0';

    if (pan_idx >= 12) {
        card->has_pan = true;
        card->protocol = CARD_PROTO_NTAG_SECURE;
    }

    // Page 8: Exp MMYY
    MfUltralightPage p8;
    if (mf_ultralight_poller_sync_read_page(nfc, 8, &p8) == MfUltralightErrorNone) {
        if (isdigit(p8.data[0]) && isdigit(p8.data[1])) {
            card->exp_month[0] = p8.data[0];
            card->exp_month[1] = p8.data[1];
            card->exp_month[2] = '\0';
        }
        if (isdigit(p8.data[2]) && isdigit(p8.data[3])) {
            snprintf(card->exp_year, sizeof(card->exp_year), "20%c%c", p8.data[2], p8.data[3]);
        }
    }

    // Page 9: CVC
    MfUltralightPage p9;
    if (mf_ultralight_poller_sync_read_page(nfc, 9, &p9) == MfUltralightErrorNone) {
        if (isdigit(p9.data[0]) && isdigit(p9.data[1]) && isdigit(p9.data[2])) {
            card->cvc[0] = p9.data[0];
            card->cvc[1] = p9.data[1];
            card->cvc[2] = p9.data[2];
            card->cvc[3] = '\0';
        }
    }

    // Pages 10..13: Holder
    char holder[17];
    memset(holder, 0, sizeof(holder));
    for (uint16_t p = 0; p < 4; p++) {
        MfUltralightPage page;
        if (mf_ultralight_poller_sync_read_page(nfc, 10 + p, &page) == MfUltralightErrorNone) {
            memcpy(&holder[p * 4], page.data, 4);
        }
    }
    size_t hlen = strlen(holder);
    while (hlen > 0 && holder[hlen - 1] == ' ') holder[--hlen] = '\0';
    if (hlen > 0) strncpy(card->cardholder, holder, sizeof(card->cardholder) - 1);

    // Pages 14..17: Token
    char tkn[17];
    memset(tkn, 0, sizeof(tkn));
    for (uint16_t p = 0; p < 4; p++) {
        MfUltralightPage page;
        if (mf_ultralight_poller_sync_read_page(nfc, 14 + p, &page) == MfUltralightErrorNone) {
            memcpy(&tkn[p * 4], page.data, 4);
        }
    }
    size_t tklen = strlen(tkn);
    while (tklen > 0 && tkn[tklen - 1] == ' ') tkn[--tklen] = '\0';
    if (tklen > 0) {
        strncpy(card->secure_token, tkn, sizeof(card->secure_token) - 1);
        card->has_token = true;
    }

    return true;
}

// ── DEDYKOWANY WĄTEK ROBOCZY NFC (BEZPIECZNY DLA STOSU ARM CORTEX-M4) ─────────
static int32_t flipson_nfc_worker(void* context) {
    FlipsonAppContext* app = (FlipsonAppContext*)context;
    FURI_LOG_I(TAG, "NFC Worker thread started with 8KB stack");

    while (app->worker_running) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        bool in_write_mode = (app->state == FlipsonStateWriteWaiting && app->write_req.active);
        FlipsonCardWriteRequest write_req = app->write_req;
        furi_mutex_release(app->mutex);

        memset(&app->data3a, 0, sizeof(app->data3a));
        Iso14443_3aError err3a = iso14443_3a_poller_sync_read(app->nfc, &app->data3a);

        if (err3a == Iso14443_3aErrorNone && app->data3a.uid_len > 0) {
            // Wykryto kartę!
            furi_mutex_acquire(app->mutex, FuriWaitForever);
            memset(&app->current_card, 0, sizeof(app->current_card));
            app->current_card.uid_len = (app->data3a.uid_len > MAX_UID_LEN) ? MAX_UID_LEN : app->data3a.uid_len;
            memcpy(app->current_card.uid_raw, app->data3a.uid, app->current_card.uid_len);
            for (size_t i = 0; i < app->current_card.uid_len; i++) {
                snprintf(&app->current_card.uid_hex[i * 2], 3, "%02X", app->current_card.uid_raw[i]);
            }
            uint32_t now_ticks = furi_get_tick();
            bool is_duplicate = (strcmp(app->last_uid, app->current_card.uid_hex) == 0 &&
                                (now_ticks - app->last_tap_tick) < furi_ms_to_ticks(2500));
            furi_mutex_release(app->mutex);

            // ── TRYB ZAPISU KARTY (PROGRAMATOR CZYSTYCH KART) ─────────────────
            if (in_write_mode) {
                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->state = FlipsonStateReading;
                snprintf(app->status_msg, sizeof(app->status_msg), "Programowanie karty...");
                furi_mutex_release(app->mutex);

                bool write_ok = false;
                // 1. Sprawdź czy to MIFARE Classic
                if ((app->data3a.sak & 0x08) || (app->data3a.sak & 0x18) || (app->data3a.sak == 0x01) || (app->data3a.sak == 0x09) || (app->data3a.sak == 0x88)) {
                    write_ok = flipson_mifare_write_data(app->nfc, &write_req);
                    if (write_ok) {
                        app->current_card.protocol = CARD_PROTO_MIFARE_SECURE;
                    }
                }

                // 2. Jeśli nie, sprawdź czy to MIFARE Ultralight / NTAG
                if (!write_ok) {
                    write_ok = flipson_mful_write_data(app->nfc, &write_req);
                    if (write_ok) {
                        app->current_card.protocol = CARD_PROTO_NTAG_SECURE;
                    }
                }

                // 3. Ostateczna próba zapisu Mifare Classic
                if (!write_ok) {
                    write_ok = flipson_mifare_write_data(app->nfc, &write_req);
                    if (write_ok) {
                        app->current_card.protocol = CARD_PROTO_MIFARE_SECURE;
                    }
                }

                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->write_req.active = false;
                if (write_ok) {
                    app->state = FlipsonStateWriteSuccess;
                    snprintf(app->status_msg, sizeof(app->status_msg), "ZAPISANO POMYSLNIE!");
                } else {
                    app->state = FlipsonStateError;
                    snprintf(app->status_msg, sizeof(app->status_msg), "Blad zapisu tagu");
                }
                furi_mutex_release(app->mutex);

                flipson_transmit_write_ack(app, &app->current_card, write_ok);

                if (write_ok) {
                    flipson_play_write_success_chime();
                } else {
                    flipson_play_error_buzz();
                }

                // Czekaj aż użytkownik odsunie kartę od anteny (maks. 2.5s)
                uint32_t wait_start = furi_get_tick();
                while (app->worker_running && (furi_get_tick() - wait_start) < furi_ms_to_ticks(2500)) {
                    memset(&app->dummy3a, 0, sizeof(app->dummy3a));
                    if (iso14443_3a_poller_sync_read(app->nfc, &app->dummy3a) != Iso14443_3aErrorNone) {
                        break;
                    }
                    furi_delay_ms(100);
                }
                furi_delay_ms(200);

                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->state = FlipsonStateWaiting;
                snprintf(app->status_msg, sizeof(app->status_msg), "Zbliz karte do Flippera");
                furi_mutex_release(app->mutex);
                continue;
            }

            // ── TRYB ODCZYTU KARTY (TERMINAL POS) ─────────────────────────────
            if (is_duplicate) {
                furi_delay_ms(100);
                continue;
            }

            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->state = FlipsonStateReading;
            snprintf(app->status_msg, sizeof(app->status_msg), "Odczyt danych karty...");
            furi_mutex_release(app->mutex);

            // 1. Sprawdź czy to karta z zapisanymi danymi MIFARE Classic
            bool has_custom = false;
            if ((app->data3a.sak & 0x08) || (app->data3a.sak & 0x18)) {
                has_custom = flipson_mifare_read_custom_data(app->nfc, &app->current_card);
            }

            // 2. Sprawdź czy to karta z zapisanymi danymi MIFARE Ultralight / NTAG
            if (!has_custom) {
                has_custom = flipson_mful_read_custom_data(app->nfc, &app->current_card);
            }

            // 3. Jeśli nie ma danych własnych i karta obsługuje ISO14443-4 -> EMV (Visa, Mastercard, Apple/Google Pay)
            if (!has_custom && iso14443_3a_supports_iso14443_4(&app->data3a)) {
                app->current_card.protocol = CARD_PROTO_EMV_GENERIC;
                flipson_read_emv_4a(app, &app->current_card);
            }

            // 4. Jeśli nadal brak protokołu -> przypisz typ tagu na podstawie SAK
            if (app->current_card.protocol == CARD_PROTO_UNKNOWN) {
                if ((app->data3a.sak & 0x08) || (app->data3a.sak & 0x18)) {
                    app->current_card.protocol = CARD_PROTO_MIFARE_CLASSIC;
                } else if (app->data3a.sak == 0x00) {
                    app->current_card.protocol = CARD_PROTO_MIFARE_ULTRALIGHT;
                } else {
                    app->current_card.protocol = CARD_PROTO_NFC_TAG;
                }
            }

            if (!app->current_card.has_token && strlen(app->current_card.uid_hex) > 0) {
                snprintf(app->current_card.secure_token, sizeof(app->current_card.secure_token), "POS_SEC_%s", app->current_card.uid_hex);
                app->current_card.has_token = true;
            }

            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->tap_count++;
            app->last_tap_tick = now_ticks;
            strncpy(app->last_uid, app->current_card.uid_hex, sizeof(app->last_uid) - 1);
            app->state = FlipsonStateSuccess;
            snprintf(app->status_msg, sizeof(app->status_msg), "WYSLANO DO KASY!");
            furi_mutex_release(app->mutex);

            flipson_transmit_card_json(app, &app->current_card);
            flipson_play_pos_chime();

            // Pętla usunięcia karty: czekaj aż karta zostanie fizycznie zabrana (maks. 2.5s)
            uint32_t wait_start = furi_get_tick();
            while (app->worker_running && (furi_get_tick() - wait_start) < furi_ms_to_ticks(2500)) {
                memset(&app->dummy3a, 0, sizeof(app->dummy3a));
                if (iso14443_3a_poller_sync_read(app->nfc, &app->dummy3a) != Iso14443_3aErrorNone) {
                    break;
                }
                furi_delay_ms(100);
            }
            furi_delay_ms(200);

            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->state = FlipsonStateWaiting;
            snprintf(app->status_msg, sizeof(app->status_msg), "Zbliz karte do Flippera");
            furi_mutex_release(app->mutex);
        } else {
            furi_delay_ms(40);
        }
    }

    FURI_LOG_I(TAG, "NFC Worker thread exiting cleanly");
    return 0;
}

// ── POMOCNICZE PARSOWANIE JSON Z DOWOLNYMI BIAŁYMI ZNAKAMI ────────────────────
static const char* flipson_find_json_val_start(const char* json, const char* key) {
    if (!json || !key) return NULL;
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return NULL;
    p += strlen(pat);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != ':') return NULL;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p == '\"') p++;
    return p;
}

static void flipson_copy_json_str(const char* src, char* dst, size_t dst_size) {
    if (!src || !dst || dst_size == 0) return;
    size_t i = 0;
    while (*src && *src != '\"' && *src != '\r' && *src != '\n' && *src != ',' && *src != '}' && i < dst_size - 1) {
        dst[i++] = *src++;
    }
    dst[i] = '\0';
}

// ── PARSER KOMEND PRZYCHODZĄCYCH Z KASY / PROGRAMATORA (WRITE_CARD) ──────────
static void flipson_process_incoming_command(FlipsonAppContext* app, const char* json) {
    if (!app || !json) return;

    if (strstr(json, "WRITE_CARD") || strstr(json, "\"pan\"") || strstr(json, "write")) {
        FlipsonCardWriteRequest req;
        memset(&req, 0, sizeof(req));
        req.active = true;

        const char* p_pan = flipson_find_json_val_start(json, "pan");
        if (p_pan) flipson_copy_json_str(p_pan, req.pan, sizeof(req.pan));

        const char* p_exp = flipson_find_json_val_start(json, "exp");
        if (p_exp) {
            char raw_exp[16] = {0};
            flipson_copy_json_str(p_exp, raw_exp, sizeof(raw_exp));
            size_t e_idx = 0;
            for (size_t i = 0; raw_exp[i] && e_idx < sizeof(req.exp) - 1; i++) {
                if (isdigit((unsigned char)raw_exp[i])) req.exp[e_idx++] = raw_exp[i];
            }
            req.exp[e_idx] = '\0';
        }

        const char* p_cvc = flipson_find_json_val_start(json, "cvc");
        if (p_cvc) flipson_copy_json_str(p_cvc, req.cvc, sizeof(req.cvc));

        const char* p_holder = flipson_find_json_val_start(json, "holder");
        if (p_holder) flipson_copy_json_str(p_holder, req.cardholder, sizeof(req.cardholder));

        const char* p_tkn = flipson_find_json_val_start(json, "token");
        if (p_tkn) flipson_copy_json_str(p_tkn, req.secure_token, sizeof(req.secure_token));

        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->write_req = req;
        app->state = FlipsonStateWriteWaiting;
        snprintf(app->status_msg, sizeof(app->status_msg), "Zbliz czysta karte...");
        furi_mutex_release(app->mutex);

        FURI_LOG_I(TAG, "Switched to WRITE MODE for PAN=%s Holder=%s", req.pan, req.cardholder);
    }
}

// Sprawdzanie wejścia szeregowego (bezpieczny odczyt ze standardowego wejścia stdin VCP)
static void flipson_check_serial_input(FlipsonAppContext* app) {
    if (!app) return;
    char stdin_ch;
    while (furi_thread_stdin_read(&stdin_ch, 1, 0) == 1) {
        if (stdin_ch == '\n' || stdin_ch == '\r') {
            if (app->rx_line_len > 0) {
                app->rx_line_buf[app->rx_line_len] = '\0';
                flipson_process_incoming_command(app, app->rx_line_buf);
                app->rx_line_len = 0;
            }
        } else if (app->rx_line_len < sizeof(app->rx_line_buf) - 1) {
            app->rx_line_buf[app->rx_line_len++] = stdin_ch;
        }
    }
}

// Sygnał systemowy zakończenia aplikacji (FuriSignalExit) wywoływany przez OS lub "loader close"
static bool flipson_thread_signal_callback(uint32_t signal, void* arg, void* context) {
    UNUSED(arg);
    FlipsonAppContext* app = (FlipsonAppContext*)context;
    if (signal == FuriSignalExit) {
        if (app) {
            app->running = false;
        }
        return true;
    }
    return false;
}

// Obsługa dedykowanej komendy CLI Flippera: "flipson <write|cancel|ping|exit>"
static void flipson_cli_callback(PipeSide* pipe, FuriString* args, void* context) {
    UNUSED(pipe);
    FlipsonAppContext* app = (FlipsonAppContext*)context;
    if (!app || !args) return;

    const char* str = furi_string_get_cstr(args);
    FURI_LOG_I(TAG, "CLI command received: %s", str);

    if (strncmp(str, "write", 5) == 0) {
        const char* json_part = strchr(str, '{');
        if (json_part) {
            flipson_process_incoming_command(app, json_part);
            printf("{\"status\":\"OK\",\"message\":\"Flipson in write mode\"}\r\n");
        } else {
            printf("{\"status\":\"ERROR\",\"message\":\"No JSON payload provided\"}\r\n");
        }
    } else if (strncmp(str, "cancel", 6) == 0) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->state = FlipsonStateWaiting;
        app->write_req.active = false;
        snprintf(app->status_msg, sizeof(app->status_msg), "Zbliz karte do Flippera");
        furi_mutex_release(app->mutex);
        printf("{\"status\":\"OK\",\"message\":\"Write cancelled\"}\r\n");
    } else if (strncmp(str, "exit", 4) == 0) {
        app->running = false;
        printf("{\"status\":\"OK\",\"message\":\"Exiting Flipson POS\"}\r\n");
    } else if (strncmp(str, "ping", 4) == 0) {
        printf("{\"status\":\"OK\",\"app\":\"FLIPSON POS\",\"state\":%d}\r\n", app->state);
    } else {
        printf("Usage: flipson <write {...}|cancel|ping|exit>\r\n");
    }
}

// ── RYSOWANIE INTERFEJSU NA EKRANIE LCD FLIPPERA (128x64) ────────────────────
static void flipson_render_callback(Canvas* canvas, void* ctx) {
    FlipsonAppContext* app = (FlipsonAppContext*)ctx;
    furi_mutex_acquire(app->mutex, FuriWaitForever);

    canvas_clear(canvas);

    // Górna belka
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_box(canvas, 0, 0, 128, 12);
    canvas_set_color(canvas, ColorWhite);
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2, 9, "FLIPSON POS");

    if (app->state == FlipsonStateWriteWaiting) {
        canvas_draw_str(canvas, 66, 9, "[PROGRAMATOR]");
    } else {
        canvas_draw_str(canvas, 76, 9, "USB & UART");
    }

    canvas_set_color(canvas, ColorBlack);

    if (app->state == FlipsonStateWaiting) {
        // Ekran oczekiwania na zbliżenie (Reader)
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 26, "ZBLIZ KARTE NFC");

        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 6, 38, "EMV / Czysta Karta / NFC");
        canvas_draw_str(canvas, 6, 48, "lub Apple / Google Pay");

        canvas_draw_line(canvas, 0, 52, 128, 52);
        char stats_str[48];
        snprintf(stats_str, sizeof(stats_str), "Odczytow: %lu | OK", (unsigned long)app->tap_count);
        canvas_draw_str(canvas, 4, 62, stats_str);

        // Ikona fali radiowej
        canvas_draw_circle(canvas, 115, 30, 4);
        canvas_draw_circle(canvas, 115, 30, 8);
        canvas_draw_circle(canvas, 115, 30, 12);

    } else if (app->state == FlipsonStateWriteWaiting) {
        // Ekran programowania czystej karty
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 4, 25, "ZBLIZ CZYSTA KARTE");

        canvas_set_font(canvas, FontSecondary);
        if (strlen(app->write_req.pan) >= 12) {
            char mpan[24];
            size_t plen = strlen(app->write_req.pan);
            snprintf(mpan, sizeof(mpan), "PAN: %.4s **** %.4s", app->write_req.pan, &app->write_req.pan[plen - 4]);
            canvas_draw_str(canvas, 4, 38, mpan);
        } else {
            canvas_draw_str(canvas, 4, 38, "Programowanie sektorow");
        }
        canvas_draw_str(canvas, 4, 48, "Mifare / NTAG / Sektor 1");

        canvas_draw_line(canvas, 0, 52, 128, 52);
        canvas_draw_str(canvas, 4, 62, "Anuluj: Wstecz [Back]");

    } else if (app->state == FlipsonStateWriteSuccess) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 28, "ZAPISANO POMYSLNIE!");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 6, 42, "Karta gotowa do platnosci");
        canvas_draw_line(canvas, 0, 52, 128, 52);
        canvas_draw_str(canvas, 4, 62, "Zdejmij karte z pleckow");

    } else if (app->state == FlipsonStateReading) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 10, 30, "ODCZYT SEKTOROW...");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 10, 45, "Trzymaj przy pleckach");

    } else if (app->state == FlipsonStateSuccess) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 4, 24, "WYSLANO DO KASY!");

        canvas_set_font(canvas, FontSecondary);
        if (app->current_card.has_pan) {
            char masked[32];
            size_t plen = strlen(app->current_card.pan);
            snprintf(masked, sizeof(masked), "%.4s **** %.4s", app->current_card.pan, &app->current_card.pan[plen - 4]);
            canvas_draw_str(canvas, 4, 36, masked);
        } else {
            char uid_str[48];
            snprintf(uid_str, sizeof(uid_str), "UID: %s", app->current_card.uid_hex);
            canvas_draw_str(canvas, 4, 36, uid_str);
        }

        char proto_str[48];
        snprintf(proto_str, sizeof(proto_str), "TYP: %s", card_protocol_name(app->current_card.protocol));
        canvas_draw_str(canvas, 4, 48, proto_str);

        canvas_draw_line(canvas, 0, 52, 128, 52);
        canvas_draw_str(canvas, 4, 62, "Zdejmij karte z pleckow ->");

    } else if (app->state == FlipsonStateError) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 10, 28, "BLAD OPERACJI!");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 10, 42, app->status_msg);
        canvas_draw_line(canvas, 0, 52, 128, 52);
        canvas_draw_str(canvas, 4, 62, "Sprobuj ponownie ->");
    }

    furi_mutex_release(app->mutex);
}

static void flipson_input_callback(InputEvent* input_event, void* ctx) {
    FuriMessageQueue* event_queue = (FuriMessageQueue*)ctx;
    furi_message_queue_put(event_queue, input_event, FuriWaitForever);
}

// ── PUNKT WEJŚCIA APLIKACJI FLIPPER ZERO ─────────────────────────────────────
int32_t flipson_pos_app(void* p) {
    UNUSED(p);
    FURI_LOG_I(TAG, "Starting FLIPSON POS Reader & Card Writer App");

    FlipsonAppContext* app = malloc(sizeof(FlipsonAppContext));
    if (!app) {
        FURI_LOG_E(TAG, "Failed to allocate FlipsonAppContext");
        return -1;
    }
    memset(app, 0, sizeof(FlipsonAppContext));
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->state = FlipsonStateWaiting;
    snprintf(app->status_msg, sizeof(app->status_msg), "Gotowy");

    // Inicjalizacja portu szeregowego UART na GPIO (Pin 13 TX, 115200 baud)
    app->serial_handle = furi_hal_serial_control_acquire(FuriHalSerialIdUsart);
    if (app->serial_handle) {
        furi_hal_serial_init(app->serial_handle, 115200);
        FURI_LOG_I(TAG, "USART acquired and initialized at 115200 baud");
    } else {
        FURI_LOG_W(TAG, "Could not acquire USART handle (might be busy)");
    }

    FuriMessageQueue* event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, flipson_render_callback, app);
    view_port_input_callback_set(view_port, flipson_input_callback, event_queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    // Inicjalizacja NFC
    app->nfc = nfc_alloc();

    // Rejestracja obsługi sygnałów systemowych (graceful exit na loader close)
    app->running = true;
    furi_thread_set_signal_callback(furi_thread_get_current(), flipson_thread_signal_callback, app);

    // Rejestracja komendy CLI 'flipson' w powłoce systemowej Flippera
    CliRegistry* cli = furi_record_open(RECORD_CLI);
    if (cli) {
        cli_registry_add_command(cli, "flipson", CliCommandFlagParallelSafe | CliCommandFlagInsomniaSafe, flipson_cli_callback, app);
        FURI_LOG_I(TAG, "Registered 'flipson' CLI command (ParallelSafe)");
    }

    // Start dedykowanego wątku roboczego NFC z bezpiecznym stosem 8KB (chroni przed MPU Fault)
    app->worker_running = true;
    app->nfc_worker_thread = furi_thread_alloc_ex("FlipsonNfc", NFC_THREAD_STACK_SIZE, flipson_nfc_worker, app);
    furi_thread_start(app->nfc_worker_thread);

    InputEvent event;
    while (app->running) {
        // Sprawdź przychodzące komendy ze standardowego wejścia stdin VCP (np. WRITE_CARD)
        flipson_check_serial_input(app);

        if (furi_message_queue_get(event_queue, &event, 50) == FuriStatusOk) {
            if (event.type == InputTypeShort) {
                if (event.key == InputKeyBack) {
                    if (app->state == FlipsonStateWriteWaiting) {
                        // Anuluj tryb zapisu, wróć do czytnika
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        app->state = FlipsonStateWaiting;
                        app->write_req.active = false;
                        snprintf(app->status_msg, sizeof(app->status_msg), "Zbliz karte do Flippera");
                        furi_mutex_release(app->mutex);
                    } else {
                        app->running = false;
                    }
                }
            }
        }
        view_port_update(view_port);
    }

    // Wyrejestrowanie komendy CLI
    if (cli) {
        cli_registry_delete_command(cli, "flipson");
        furi_record_close(RECORD_CLI);
    }

    // Zatrzymanie wątku roboczego NFC
    app->worker_running = false;
    furi_thread_join(app->nfc_worker_thread);
    furi_thread_free(app->nfc_worker_thread);

    // Sprzątanie NFC
    nfc_free(app->nfc);

    // Sprzątanie interfejsu graficznego
    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_record_close(RECORD_GUI);
    furi_message_queue_free(event_queue);

    // Zwolnienie portu UART
    if (app->serial_handle) {
        furi_hal_serial_deinit(app->serial_handle);
        furi_hal_serial_control_release(app->serial_handle);
        app->serial_handle = NULL;
    }

    furi_mutex_free(app->mutex);
    free(app);

    FURI_LOG_I(TAG, "FLIPSON POS Reader Exited cleanly");
    return 0;
}
