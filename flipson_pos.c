/**
 * FLIPSON POS Reader & Card Writer — Zaawansowana Aplikacja Flipper Zero
 * Pełna emulacja terminala płatniczego:
 *  - Wyświetlanie kwoty transakcji ("DO ZAPŁATY: 49.99 PLN")
 *  - Czytelne instrukcje na ekranie LCD ("PROSZE ZBLIŻYĆ KARTĘ")
 *  - Wprowadzanie kodu PIN bezpośrednio na Flipperze strzałkami (klawiatura ekranowa 0-9, <, OK)
 *  - Ekrany statusu transakcji ("TRANSAKCJA PRZYJĘTA" / "TRANSAKCJA ODRZUCONA")
 *  - Zapis i odczyt kart bankowych na czystych tagach NFC (MIFARE Classic / Ultralight / NTAG)
 *  - Bezpieczna dwukierunkowa transmisja przez USB VCP CLI Shell oraz UART GPIO Pin 13
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
#include <toolbox/cli/cli_registry.h>

#include "flipson_emv.h"

#define RECORD_CLI "cli"
#define TAG "FlipsonPos"
#define NFC_THREAD_STACK_SIZE (8 * 1024)

typedef enum {
    FlipsonStateWaiting,          // Oczekiwanie na kartę w trybie czytnika POS
    FlipsonStatePaymentPending,   // Terminal: kwota do zapłaty ("DO ZAPŁATY: 49.99 PLN")
    FlipsonStatePinEntry,         // Ekran wprowadzania PIN strzałkami i zatwierdzania OK
    FlipsonStateReading,          // Czytanie APDU / sektorów
    FlipsonStateSuccess,          // Sukces odczytu karty
    FlipsonStatePaymentApproved,  // Płatność zaakceptowana ("TRANSAKCJA PRZYJĘTA")
    FlipsonStatePaymentDeclined,  // Płatność odrzucona ("TRANSAKCJA ODRZUCONA")
    FlipsonStateWriteWaiting,     // Programator: oczekiwanie na czystą kartę do zapisu
    FlipsonStateWriteSuccess,     // Sukces programowania czystej karty
    FlipsonStateError             // Błąd operacji
} FlipsonState;

typedef struct {
    FuriMutex* mutex;
    FlipsonState state;
    FlipsonCardData current_card;
    FlipsonCardWriteRequest write_req;
    uint32_t tap_count;
    FlipsonCardData last_card;
    char last_uid[32];
    uint32_t last_tap_tick;
    char status_msg[64];

    // Dane terminala i płatności
    char payment_amount[32];      // np. "49.99 PLN"
    char payment_msg[48];         // np. "Zatwierdzono" / "Brak srodkow"
    char entered_pin[8];          // Wpisany PIN (do 6 cyfr)
    size_t entered_pin_len;
    int pin_cursor_pos;           // 0..11: 0-9 (cyfry '0'-'9'), 10 ('<'), 11 ('OK')
    uint32_t result_screen_tick;  // Tick wyświetlenia ekranu sukcesu/błędu do auto-powrotu

    // Bufor zdarzeń dla strumieniowania CLI Session
    char pending_event_json[512];
    bool has_pending_event;
    uint32_t event_seq;

    FuriHalSerialHandle* serial_handle;
    Nfc* nfc;
    FuriThread* nfc_worker_thread;
    volatile bool worker_running;
    volatile bool running;

    // Bufory alokowane w stercie (ochrona stosu Cortex-M4)
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

// Zwycięski sygnał zaakceptowania płatności ("TRANSAKCJA PRZYJĘTA")
static void flipson_play_payment_approved_chime(void) {
    if (furi_hal_speaker_is_mine() || furi_hal_speaker_acquire(100)) {
        furi_hal_speaker_start(1500.0f, 0.35f);
        furi_delay_ms(80);
        furi_hal_speaker_start(2000.0f, 0.4f);
        furi_delay_ms(80);
        furi_hal_speaker_start(3000.0f, 0.5f);
        furi_delay_ms(180);
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
    }
    furi_hal_light_set(LightGreen, 255);
    furi_hal_vibro_on(true);
    furi_delay_ms(60);
    furi_hal_vibro_on(false);
    furi_delay_ms(100);
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

// Krótki klik przy naciśnięciu cyfry PIN
static void flipson_play_key_click(void) {
    if (furi_hal_speaker_is_mine() || furi_hal_speaker_acquire(50)) {
        furi_hal_speaker_start(2200.0f, 0.15f);
        furi_delay_ms(25);
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
    }
}

// Niski sygnał błędu / odrzucenia transakcji
static void flipson_play_error_buzz(void) {
    if (furi_hal_speaker_is_mine() || furi_hal_speaker_acquire(100)) {
        furi_hal_speaker_start(220.0f, 0.4f);
        furi_delay_ms(220);
        furi_hal_speaker_stop();
        furi_delay_ms(60);
        furi_hal_speaker_start(180.0f, 0.4f);
        furi_delay_ms(280);
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
    }
    furi_hal_light_set(LightRed, 255);
    furi_hal_vibro_on(true);
    furi_delay_ms(150);
    furi_hal_vibro_on(false);
    furi_delay_ms(100);
    furi_hal_light_set(LightRed, 0);
}

// Przygotowanie pakietu JSON i rejestracja do emisji przez UART i sesję CLI
static void flipson_emit_json_event(FlipsonAppContext* app, const char* json_buffer) {
    if (!app || !json_buffer) return;
    size_t len = strlen(json_buffer);

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    strncpy(app->pending_event_json, json_buffer, sizeof(app->pending_event_json) - 1);
    app->has_pending_event = true;
    app->event_seq++;
    furi_mutex_release(app->mutex);

    // Sprzętowy UART na pinie 13 GPIO (115200 baud)
    if (app->serial_handle) {
        furi_hal_serial_tx(app->serial_handle, (const uint8_t*)json_buffer, len);
        furi_hal_serial_tx_wait_complete(app->serial_handle);
    }

    FURI_LOG_I(TAG, "EVENT: %s", json_buffer);
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
        "\"exp_year\":\"%s\",\"cvc\":\"%s\",\"cardholder\":\"%s\",\"token\":\"%s\",\"amount\":\"%s\",\"ts\":%lu}\r\n",
        card_protocol_name(card->protocol),
        card->uid_hex,
        card->has_pan ? card->pan : "",
        card->exp_month,
        card->exp_year,
        card->cvc,
        card->cardholder,
        card->secure_token,
        app->payment_amount,
        (unsigned long)now_ts
    );

    flipson_emit_json_event(app, app->json_tx_buf);
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

    flipson_emit_json_event(app, app->json_tx_buf);
}

// Wysłanie wprowadzonego kodu PIN (PIN_ENTERED)
static void flipson_transmit_pin_submit(FlipsonAppContext* app) {
    if (!app) return;
    uint32_t now_ts = furi_hal_rtc_get_timestamp();
    snprintf(
        app->json_tx_buf,
        sizeof(app->json_tx_buf),
        "{\"event\":\"PIN_SUBMIT\",\"pin\":\"%s\",\"amount\":\"%s\",\"ts\":%lu}\r\n",
        app->entered_pin,
        app->payment_amount,
        (unsigned long)now_ts
    );

    flipson_emit_json_event(app, app->json_tx_buf);
}

// Wysłanie anulowania operacji (PIN_CANCEL)
static void flipson_transmit_pin_cancel(FlipsonAppContext* app) {
    if (!app) return;
    snprintf(app->json_tx_buf, sizeof(app->json_tx_buf), "{\"event\":\"PIN_CANCEL\"}\r\n");
    flipson_emit_json_event(app, app->json_tx_buf);
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
} FlipsonEmvContext;

static NfcCommand flipson_emv_poller_callback(NfcGenericEventEx event, void* context) {
    FlipsonEmvContext* emv_ctx = (FlipsonEmvContext*)context;
    Iso14443_4aPoller* poller = (Iso14443_4aPoller*)event.poller;
    if (poller && emv_ctx && emv_ctx->app && emv_ctx->card) {
        flipson_emv_perform_full_read(poller, emv_ctx->card, emv_ctx->app->apdu_rx_buf, sizeof(emv_ctx->app->apdu_rx_buf));
    }
    return NfcCommandStop;
}

static void flipson_read_emv_4a(FlipsonAppContext* app, FlipsonCardData* card) {
    if (!app || !card) return;
    NfcPoller* poller = nfc_poller_alloc(app->nfc, NfcProtocolIso14443_4a);
    if (!poller) return;

    FlipsonEmvContext emv_ctx = {.app = app, .card = card};
    nfc_poller_start_ex(poller, flipson_emv_poller_callback, &emv_ctx);
    nfc_poller_stop(poller);
    nfc_poller_free(poller);
}

// ── ZAPIS I ODCZYT MIFARE CLASSIC (Sektor 1: Bloki 4, 5, 6) ───────────────────
static const MfClassicKey DEFAULT_KEYS[] = {
    {.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}},
    {.data = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5}},
    {.data = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7}},
    {.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}
};
#define NUM_DEFAULT_KEYS (sizeof(DEFAULT_KEYS) / sizeof(DEFAULT_KEYS[0]))

static bool flipson_mifare_auth_sector1(Nfc* nfc, MfClassicKey* out_key, MfClassicKeyType* out_type) {
    MfClassicAuthContext auth_ctx;
    for (size_t k = 0; k < NUM_DEFAULT_KEYS; k++) {
        MfClassicKey test_key = DEFAULT_KEYS[k];
        if (mf_classic_poller_sync_auth(nfc, 4, &test_key, MfClassicKeyTypeA, &auth_ctx) == MfClassicErrorNone) {
            *out_key = DEFAULT_KEYS[k];
            *out_type = MfClassicKeyTypeA;
            return true;
        }
        test_key = DEFAULT_KEYS[k];
        if (mf_classic_poller_sync_auth(nfc, 4, &test_key, MfClassicKeyTypeB, &auth_ctx) == MfClassicErrorNone) {
            *out_key = DEFAULT_KEYS[k];
            *out_type = MfClassicKeyTypeB;
            return true;
        }
    }
    return false;
}

static bool flipson_mifare_write_data(Nfc* nfc, const FlipsonCardWriteRequest* req) {
    if (!nfc || !req) return false;
    MfClassicKey active_key;
    MfClassicKeyType active_key_type;

    if (!flipson_mifare_auth_sector1(nfc, &active_key, &active_key_type)) {
        return false;
    }

    // Blok 4: 16 cyfr PAN
    MfClassicBlock b4;
    memset(b4.data, ' ', 16);
    size_t plen = strlen(req->pan);
    if (plen > 16) plen = 16;
    memcpy(b4.data, req->pan, plen);
    if (mf_classic_poller_sync_write_block(nfc, 4, &active_key, active_key_type, &b4) != MfClassicErrorNone) {
        return false;
    }

    // Blok 5: Exp (4) + CVC (3) + Cardholder (9) = 16 bajtów
    MfClassicBlock b5;
    memset(b5.data, ' ', 16);
    size_t elen = strlen(req->exp);
    if (elen > 4) elen = 4;
    memcpy(&b5.data[0], req->exp, elen);

    size_t clen = strlen(req->cvc);
    if (clen > 3) clen = 3;
    memcpy(&b5.data[4], req->cvc, clen);

    size_t hlen = strlen(req->cardholder);
    if (hlen > 9) hlen = 9;
    memcpy(&b5.data[7], req->cardholder, hlen);

    if (mf_classic_poller_sync_write_block(nfc, 5, &active_key, active_key_type, &b5) != MfClassicErrorNone) {
        return false;
    }

    // Blok 6: Token bezpieczeństwa (16 bajtów)
    MfClassicBlock b6;
    memset(b6.data, ' ', 16);
    size_t tlen = strlen(req->secure_token);
    if (tlen > 16) tlen = 16;
    memcpy(b6.data, req->secure_token, tlen);

    if (mf_classic_poller_sync_write_block(nfc, 6, &active_key, active_key_type, &b6) != MfClassicErrorNone) {
        return false;
    }

    return true;
}

static bool flipson_mifare_read_custom_data(Nfc* nfc, FlipsonCardData* card) {
    if (!nfc || !card) return false;
    MfClassicKey active_key;
    MfClassicKeyType active_key_type;

    if (flipson_mifare_auth_sector1(nfc, &active_key, &active_key_type)) {
        MfClassicBlock b4;
        if (mf_classic_poller_sync_read_block(nfc, 4, &active_key, active_key_type, &b4) == MfClassicErrorNone) {
            size_t digits = 0;
            for (size_t i = 0; i < 16; i++) {
                if (isdigit(b4.data[i])) digits++;
            }
            if (digits >= 12) {
                size_t pan_idx = 0;
                for (size_t i = 0; i < 16 && isdigit(b4.data[i]); i++) {
                    card->pan[pan_idx++] = b4.data[i];
                }
                card->pan[pan_idx] = '\0';
                card->has_pan = true;
                card->protocol = CARD_PROTO_MIFARE_SECURE;

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
                    if (nlen > 0) strncpy(card->cardholder, raw_name, sizeof(card->cardholder) - 1);
                }

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

// ── ZAPIS I ODCZYT MIFARE ULTRALIGHT / NTAG ────────────────────────────────────
static bool flipson_mful_write_data(Nfc* nfc, const FlipsonCardWriteRequest* req) {
    if (!nfc || !req) return false;

    // Pages 4..7: PAN 16 cyfr
    char pan16[17] = "                ";
    size_t plen = strlen(req->pan);
    if (plen > 16) plen = 16;
    memcpy(pan16, req->pan, plen);

    for (uint16_t p = 0; p < 4; p++) {
        MfUltralightPage page;
        memcpy(page.data, &pan16[p * 4], 4);
        if (mf_ultralight_poller_sync_write_page(nfc, 4 + p, &page) != MfUltralightErrorNone) return false;
    }

    // Page 8: Exp (MMYY)
    char exp4[5] = "    ";
    size_t elen = strlen(req->exp);
    if (elen > 4) elen = 4;
    memcpy(exp4, req->exp, elen);
    MfUltralightPage p8;
    memcpy(p8.data, exp4, 4);
    if (mf_ultralight_poller_sync_write_page(nfc, 8, &p8) != MfUltralightErrorNone) return false;

    // Page 9: CVC (3 cyfry + spacja)
    char cvc4[5] = "    ";
    size_t clen = strlen(req->cvc);
    if (clen > 3) clen = 3;
    memcpy(cvc4, req->cvc, clen);
    MfUltralightPage p9;
    memcpy(p9.data, cvc4, 4);
    if (mf_ultralight_poller_sync_write_page(nfc, 9, &p9) != MfUltralightErrorNone) return false;

    // Pages 10..13: Holder
    char holder16[17] = "                ";
    size_t hlen = strlen(req->cardholder);
    if (hlen > 16) hlen = 16;
    memcpy(holder16, req->cardholder, hlen);
    for (uint16_t p = 0; p < 4; p++) {
        MfUltralightPage page;
        memcpy(page.data, &holder16[p * 4], 4);
        if (mf_ultralight_poller_sync_write_page(nfc, 10 + p, &page) != MfUltralightErrorNone) return false;
    }

    // Pages 14..17: Token
    char token16[17] = "                ";
    size_t tlen = strlen(req->secure_token);
    if (tlen > 16) tlen = 16;
    memcpy(token16, req->secure_token, tlen);
    for (uint16_t p = 0; p < 4; p++) {
        MfUltralightPage page;
        memcpy(page.data, &token16[p * 4], 4);
        if (mf_ultralight_poller_sync_write_page(nfc, 14 + p, &page) != MfUltralightErrorNone) return false;
    }

    return true;
}

static bool flipson_mful_read_custom_data(Nfc* nfc, FlipsonCardData* card) {
    if (!nfc || !card) return false;

    MfUltralightPage p4;
    if (mf_ultralight_poller_sync_read_page(nfc, 4, &p4) != MfUltralightErrorNone) return false;
    if (!isdigit(p4.data[0]) || !isdigit(p4.data[1])) return false;

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

    MfUltralightPage p9;
    if (mf_ultralight_poller_sync_read_page(nfc, 9, &p9) == MfUltralightErrorNone) {
        if (isdigit(p9.data[0]) && isdigit(p9.data[1]) && isdigit(p9.data[2])) {
            card->cvc[0] = p9.data[0];
            card->cvc[1] = p9.data[1];
            card->cvc[2] = p9.data[2];
            card->cvc[3] = '\0';
        }
    }

    char holder[17] = {0};
    for (uint16_t p = 0; p < 4; p++) {
        MfUltralightPage page;
        if (mf_ultralight_poller_sync_read_page(nfc, 10 + p, &page) == MfUltralightErrorNone) {
            memcpy(&holder[p * 4], page.data, 4);
        }
    }
    size_t hlen = strlen(holder);
    while (hlen > 0 && holder[hlen - 1] == ' ') holder[--hlen] = '\0';
    if (hlen > 0) strncpy(card->cardholder, holder, sizeof(card->cardholder) - 1);

    char tkn[17] = {0};
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

// ── WĄTEK ROBOCZY NFC (BEZPIECZNY DLA STOSU 8KB) ──────────────────────────────
static int32_t flipson_nfc_worker(void* context) {
    FlipsonAppContext* app = (FlipsonAppContext*)context;
    FURI_LOG_I(TAG, "NFC Worker started");

    while (app->worker_running) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        bool in_write_mode = (app->state == FlipsonStateWriteWaiting && app->write_req.active);
        FlipsonCardWriteRequest write_req = app->write_req;
        FlipsonState current_state = app->state;
        furi_mutex_release(app->mutex);

        // Nie próbuj odczytu NFC podczas wprowadzania PIN-u na ekranie
        if (current_state == FlipsonStatePinEntry || 
            current_state == FlipsonStatePaymentApproved || 
            current_state == FlipsonStatePaymentDeclined) {
            furi_delay_ms(100);
            continue;
        }

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
                                (now_ticks - app->last_tap_tick) < furi_ms_to_ticks(2000));
            furi_mutex_release(app->mutex);

            // ── TRYB ZAPISU KARTY ─────────────────────────────────────────────
            if (in_write_mode) {
                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->state = FlipsonStateReading;
                snprintf(app->status_msg, sizeof(app->status_msg), "Programowanie...");
                furi_mutex_release(app->mutex);

                bool write_ok = false;
                if ((app->data3a.sak & 0x08) || (app->data3a.sak & 0x18)) {
                    write_ok = flipson_mifare_write_data(app->nfc, &write_req);
                    if (write_ok) app->current_card.protocol = CARD_PROTO_MIFARE_SECURE;
                }
                if (!write_ok) {
                    write_ok = flipson_mful_write_data(app->nfc, &write_req);
                    if (write_ok) app->current_card.protocol = CARD_PROTO_NTAG_SECURE;
                }
                if (!write_ok) {
                    write_ok = flipson_mifare_write_data(app->nfc, &write_req);
                    if (write_ok) app->current_card.protocol = CARD_PROTO_MIFARE_SECURE;
                }

                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->write_req.active = false;
                if (write_ok) {
                    app->state = FlipsonStateWriteSuccess;
                    snprintf(app->status_msg, sizeof(app->status_msg), "ZAPISANO POMYSLNIE!");
                } else {
                    app->state = FlipsonStateError;
                    snprintf(app->status_msg, sizeof(app->status_msg), "Blad zapisu");
                }
                furi_mutex_release(app->mutex);

                flipson_transmit_write_ack(app, &app->current_card, write_ok);

                if (write_ok) flipson_play_write_success_chime();
                else flipson_play_error_buzz();

                // Czekaj na usunięcie karty
                uint32_t wait_start = furi_get_tick();
                while (app->worker_running && (furi_get_tick() - wait_start) < furi_ms_to_ticks(1000)) {
                    memset(&app->dummy3a, 0, sizeof(app->dummy3a));
                    if (iso14443_3a_poller_sync_read(app->nfc, &app->dummy3a) != Iso14443_3aErrorNone) break;
                    furi_delay_ms(80);
                }

                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->state = FlipsonStateWaiting;
                app->last_uid[0] = '\0';
                snprintf(app->status_msg, sizeof(app->status_msg), "Gotowy");
                furi_mutex_release(app->mutex);
                continue;
            }

            // ── TRYB ODCZYTU KARTY (TERMINAL POS) ─────────────────────────────
            if (is_duplicate) {
                furi_delay_ms(80);
                continue;
            }

            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->state = FlipsonStateReading;
            snprintf(app->status_msg, sizeof(app->status_msg), "Odczyt danych...");
            furi_mutex_release(app->mutex);

            bool has_custom = false;
            if ((app->data3a.sak & 0x08) || (app->data3a.sak & 0x18)) {
                has_custom = flipson_mifare_read_custom_data(app->nfc, &app->current_card);
            }
            if (!has_custom) {
                has_custom = flipson_mful_read_custom_data(app->nfc, &app->current_card);
            }
            if (!has_custom && iso14443_3a_supports_iso14443_4(&app->data3a)) {
                app->current_card.protocol = CARD_PROTO_EMV_GENERIC;
                flipson_read_emv_4a(app, &app->current_card);
            }
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
            app->last_card = app->current_card;
            app->last_tap_tick = now_ticks;
            strncpy(app->last_uid, app->current_card.uid_hex, sizeof(app->last_uid) - 1);
            app->state = FlipsonStateSuccess;
            snprintf(app->status_msg, sizeof(app->status_msg), "WYSLANO DO KASY!");
            furi_mutex_release(app->mutex);

            flipson_transmit_card_json(app, &app->current_card);
            flipson_play_pos_chime();

            // Czekaj na usunięcie karty
            uint32_t wait_start = furi_get_tick();
            while (app->worker_running && (furi_get_tick() - wait_start) < furi_ms_to_ticks(1000)) {
                memset(&app->dummy3a, 0, sizeof(app->dummy3a));
                if (iso14443_3a_poller_sync_read(app->nfc, &app->dummy3a) != Iso14443_3aErrorNone) break;
                furi_delay_ms(80);
            }

            furi_mutex_acquire(app->mutex, FuriWaitForever);
            float amt_f = strtof(app->payment_amount, NULL);
            // Kwota > 100 PLN wymaga podania kodu PIN na Flipperze
            if (amt_f > 100.0f) {
                app->state = FlipsonStatePinEntry;
                memset(app->entered_pin, 0, sizeof(app->entered_pin));
                app->entered_pin_len = 0;
                app->pin_cursor_pos = 0;
                snprintf(app->status_msg, sizeof(app->status_msg), "Wprowadz PIN (>100 zl)");
            } else if (strlen(app->payment_amount) > 0) {
                app->state = FlipsonStatePaymentPending;
                snprintf(app->status_msg, sizeof(app->status_msg), "Gotowy");
            } else {
                app->state = FlipsonStateWaiting;
                snprintf(app->status_msg, sizeof(app->status_msg), "Gotowy");
            }
            furi_mutex_release(app->mutex);
        } else {
            furi_delay_ms(40);
        }
    }

    FURI_LOG_I(TAG, "NFC Worker exiting");
    return 0;
}

// ── POMOCNICZE PARSOWANIE JSON ────────────────────────────────────────────────
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

// ── PRZETWARZANIE PRZYCHODZĄCYCH POLECEŃ (POS TERMINAL & PROGRAMATOR) ─────────
static void flipson_process_incoming_command(FlipsonAppContext* app, const char* json) {
    if (!app || !json) return;

    // 1. Polecenie START_PAYMENT — wyświetlenie kwoty na ekranie Flippera
    if (strstr(json, "START_PAYMENT") || strstr(json, "\"amount\"") || strstr(json, "pay")) {
        char amt[32] = {0};
        const char* p_amt = flipson_find_json_val_start(json, "amount");
        if (p_amt) {
            flipson_copy_json_str(p_amt, amt, sizeof(amt));
        } else {
            strncpy(amt, "0.00 PLN", sizeof(amt) - 1);
        }

        furi_mutex_acquire(app->mutex, FuriWaitForever);
        strncpy(app->payment_amount, amt, sizeof(app->payment_amount) - 1);
        app->state = FlipsonStatePaymentPending;
        app->last_uid[0] = '\0';
        snprintf(app->status_msg, sizeof(app->status_msg), "Zbliz karte do terminala");
        furi_mutex_release(app->mutex);

        flipson_play_key_click();
        FURI_LOG_I(TAG, "START_PAYMENT: amount=%s", amt);
        return;
    }

    // 2. Polecenie REQUEST_PIN — prośba o podanie PIN na klawiaturze Flippera
    if (strstr(json, "REQUEST_PIN") || strstr(json, "ask_pin") || strstr(json, "\"cmd\":\"PIN\"")) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->state = FlipsonStatePinEntry;
        memset(app->entered_pin, 0, sizeof(app->entered_pin));
        app->entered_pin_len = 0;
        app->pin_cursor_pos = 0;
        snprintf(app->status_msg, sizeof(app->status_msg), "Wprowadz PIN");
        furi_mutex_release(app->mutex);

        flipson_play_key_click();
        FURI_LOG_I(TAG, "Switched to PIN ENTRY mode");
        return;
    }

    // 3. Wynik autoryzacji: PAYMENT_RESULT ("APPROVED" / "DECLINED")
    if (strstr(json, "PAYMENT_RESULT") || strstr(json, "result") || strstr(json, "AUTH_")) {
        bool approved = (strstr(json, "APPROVED") != NULL || strstr(json, "\"OK\"") != NULL || strstr(json, "SUCCESS") != NULL);
        char msg[48] = {0};
        const char* p_msg = flipson_find_json_val_start(json, "msg");
        if (p_msg) flipson_copy_json_str(p_msg, msg, sizeof(msg));

        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->result_screen_tick = furi_get_tick();
        if (approved) {
            app->state = FlipsonStatePaymentApproved;
            snprintf(app->payment_msg, sizeof(app->payment_msg), "%s", msg[0] ? msg : "AUTORYZACJA PRZYJETA");
        } else {
            app->state = FlipsonStatePaymentDeclined;
            snprintf(app->payment_msg, sizeof(app->payment_msg), "%s", msg[0] ? msg : "ODRZUCONO");
        }
        furi_mutex_release(app->mutex);

        if (approved) flipson_play_payment_approved_chime();
        else flipson_play_error_buzz();

        FURI_LOG_I(TAG, "PAYMENT_RESULT: approved=%d, msg=%s", approved, msg);
        return;
    }

    // 4. Polecenie WRITE_CARD — programowanie czystej karty
    if (strstr(json, "WRITE_CARD") || strstr(json, "\"pan\"")) {
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
        app->last_uid[0] = '\0';
        snprintf(app->status_msg, sizeof(app->status_msg), "Zbliz czysta karte...");
        furi_mutex_release(app->mutex);

        flipson_play_key_click();
        FURI_LOG_I(TAG, "Switched to WRITE MODE for PAN=%s Holder=%s", req.pan, req.cardholder);
        return;
    }

    // 5. Polecenie RESET / CANCEL
    if (strstr(json, "RESET") || strstr(json, "CANCEL")) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->state = FlipsonStateWaiting;
        app->write_req.active = false;
        app->payment_amount[0] = '\0';
        snprintf(app->status_msg, sizeof(app->status_msg), "Gotowy");
        furi_mutex_release(app->mutex);
        FURI_LOG_I(TAG, "Reset to Idle state");
    }
}

// ── OBSŁUGA POLECEŃ CLI FLIPPERA ("flipson ...") ──────────────────────────────
static void flipson_cli_callback(PipeSide* pipe, FuriString* args, void* context) {
    FlipsonAppContext* app = (FlipsonAppContext*)context;
    if (!app) return;

    const char* str = args ? furi_string_get_cstr(args) : "";
    FURI_LOG_I(TAG, "CLI args: %s", str);

    // 1. flipson session — interaktywna dwukierunkowa sesja strumieniowa ze statusem
    if (strncmp(str, "session", 7) == 0 || strncmp(str, "stream", 6) == 0) {
        printf("{\"status\":\"OK\",\"message\":\"Flipson session connected\",\"app\":\"FLIPSON POS\"}\r\n");
        fflush(stdout);

        while (app->running && pipe && !cli_is_pipe_broken_or_is_etx_next_char(pipe)) {
            // Emisja oczekującego zdarzenia z wątku NFC
            furi_mutex_acquire(app->mutex, FuriWaitForever);
            if (app->has_pending_event) {
                printf("%s", app->pending_event_json);
                fflush(stdout);
                app->has_pending_event = false;
            }
            furi_mutex_release(app->mutex);

            // Odczyt linii ze strumienia wejściowego sesji
            char rx_buf[256];
            size_t rx_len = 0;
            while (rx_len < sizeof(rx_buf) - 1) {
                int ch = getchar();
                if (ch == EOF) break;
                if (ch == '\n' || ch == '\r') {
                    if (rx_len > 0) break;
                } else {
                    rx_buf[rx_len++] = (char)ch;
                }
            }
            if (rx_len > 0) {
                rx_buf[rx_len] = '\0';
                flipson_process_incoming_command(app, rx_buf);
                printf("{\"status\":\"OK\",\"ack\":\"received\"}\r\n");
                fflush(stdout);
            }

            furi_delay_ms(30);
        }
        printf("{\"status\":\"OK\",\"message\":\"Flipson session closed\"}\r\n");
        return;
    }

    // 2. flipson write {...}
    if (strncmp(str, "write", 5) == 0) {
        const char* json_part = strchr(str, '{');
        if (json_part) {
            flipson_process_incoming_command(app, json_part);
            printf("{\"status\":\"OK\",\"message\":\"Flipson in write mode\"}\r\n");
        } else {
            printf("{\"status\":\"ERROR\",\"message\":\"No JSON payload\"}\r\n");
        }
        return;
    }

    // 3. flipson pay {...} lub flipson amount {...}
    if (strncmp(str, "pay", 3) == 0 || strncmp(str, "amount", 6) == 0) {
        const char* json_part = strchr(str, '{');
        if (json_part) {
            flipson_process_incoming_command(app, json_part);
            printf("{\"status\":\"OK\",\"message\":\"Payment amount set\"}\r\n");
        } else {
            printf("{\"status\":\"ERROR\",\"message\":\"No JSON payload\"}\r\n");
        }
        return;
    }

    // 4. flipson cmd {...}
    if (strncmp(str, "cmd", 3) == 0) {
        const char* json_part = strchr(str, '{');
        if (json_part) {
            flipson_process_incoming_command(app, json_part);
            printf("{\"status\":\"OK\",\"message\":\"Command processed\"}\r\n");
        }
        return;
    }

    // 5. flipson cancel
    if (strncmp(str, "cancel", 6) == 0) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->state = FlipsonStateWaiting;
        app->write_req.active = false;
        app->payment_amount[0] = '\0';
        snprintf(app->status_msg, sizeof(app->status_msg), "Gotowy");
        furi_mutex_release(app->mutex);
        printf("{\"status\":\"OK\",\"message\":\"Cancelled\"}\r\n");
        return;
    }

    // 6. flipson exit
    if (strncmp(str, "exit", 4) == 0) {
        app->running = false;
        printf("{\"status\":\"OK\",\"message\":\"Exiting\"}\r\n");
        return;
    }

    // 7. flipson ping (odpowiedź ze stanem i ostatnią kartą)
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    uint32_t seq = app->tap_count;
    FlipsonState cur_state = app->state;
    FlipsonCardData card = app->last_card;
    char amt[32];
    strncpy(amt, app->payment_amount, sizeof(amt) - 1);
    furi_mutex_release(app->mutex);

    if (seq > 0 && strlen(card.uid_hex) > 0) {
        printf("{\"status\":\"OK\",\"app\":\"FLIPSON POS\",\"state\":%d,\"tap_seq\":%lu,"
               "\"amount\":\"%s\",\"last_tap\":{\"protocol\":\"%s\",\"uid\":\"%s\",\"card_number\":\"%s\","
               "\"exp_month\":\"%s\",\"exp_year\":\"%s\",\"cvc\":\"%s\",\"cardholder\":\"%s\","
               "\"token\":\"%s\"}}\r\n",
               cur_state,
               (unsigned long)seq,
               amt,
               card_protocol_name(card.protocol),
               card.uid_hex,
               card.has_pan ? card.pan : "",
               card.exp_month,
               card.exp_year,
               card.cvc,
               card.cardholder,
               card.secure_token);
    } else {
        printf("{\"status\":\"OK\",\"app\":\"FLIPSON POS\",\"state\":%d,\"tap_seq\":%lu,\"amount\":\"%s\"}\r\n",
               cur_state, (unsigned long)seq, amt);
    }
}

// ── RYSOWANIE EKRANU LCD FLIPPERA ZERO (128x64) ──────────────────────────────
static void flipson_render_callback(Canvas* canvas, void* ctx) {
    FlipsonAppContext* app = (FlipsonAppContext*)ctx;
    furi_mutex_acquire(app->mutex, FuriWaitForever);

    canvas_clear(canvas);

    // ── GÓRNA BELKA TYTUŁOWA DLA KLIENTA ─────────────────────────────────────
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_box(canvas, 0, 0, 128, 12);
    canvas_set_color(canvas, ColorWhite);
    canvas_set_font(canvas, FontSecondary);

    if (app->state == FlipsonStatePaymentPending) {
        canvas_draw_str(canvas, 3, 9, "TERMINAL: ODBIERANIE PLATNOSCI");
    } else if (app->state == FlipsonStateReading) {
        canvas_draw_str(canvas, 3, 9, "CZYTNIK: ODCZYTYWANIE...");
    } else if (app->state == FlipsonStatePinEntry) {
        canvas_draw_str(canvas, 3, 9, "PINPAD: WPROWADZ KOD PIN");
    } else if (app->state == FlipsonStatePaymentApproved) {
        canvas_draw_str(canvas, 3, 9, "TRANSAKCJA PRZYJETA [OK]");
    } else if (app->state == FlipsonStatePaymentDeclined) {
        canvas_draw_str(canvas, 3, 9, "TRANSAKCJA ODRZUCONA [X]");
    } else if (app->state == FlipsonStateWriteWaiting) {
        canvas_draw_str(canvas, 3, 9, "PROGRAMATOR KART NFC");
    } else if (app->state == FlipsonStateWriteSuccess) {
        canvas_draw_str(canvas, 3, 9, "ZAPIS KARTY [SUKCES]");
    } else {
        canvas_draw_str(canvas, 3, 9, "FLIPSON PAY - CZYTNIK KLIENTA");
    }

    canvas_set_color(canvas, ColorBlack);

    // ── 1. STAN: OCZEKIWANIE NA KARTĘ (BEZ ZDEFINIOWANEJ KWOTY) ───────────────
    if (app->state == FlipsonStateWaiting) {
        canvas_draw_rframe(canvas, 1, 14, 126, 49, 2);

        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 26, "ZBLIZ KARTE / TELEFON");

        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 6, 38, "Visa, MC, Google/Apple Pay");
        canvas_draw_str(canvas, 6, 48, "Przyloz do pleckow czytnika");

        // Fale radiowe NFC z prawej
        canvas_draw_disc(canvas, 114, 38, 2);
        canvas_draw_circle(canvas, 114, 38, 5);
        canvas_draw_circle(canvas, 114, 38, 8);
        canvas_draw_circle(canvas, 114, 38, 11);

        canvas_draw_line(canvas, 1, 51, 126, 51);
        char stats_str[48];
        snprintf(stats_str, sizeof(stats_str), "Gotowy | Taps: %lu", (unsigned long)app->tap_count);
        canvas_draw_str(canvas, 6, 60, stats_str);
    }
    // ── 2. STAN: KWOTA DO ZAPŁATY (ODBIERANIE PŁATNOŚCI) ──────────────────────
    else if (app->state == FlipsonStatePaymentPending) {
        canvas_draw_rframe(canvas, 1, 14, 126, 49, 2);

        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 6, 25, "DO ZAPLATY:");

        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 38, app->payment_amount[0] ? app->payment_amount : "0.00 PLN");

        canvas_set_font(canvas, FontSecondary);
        float p_amt = strtof(app->payment_amount, NULL);
        if (p_amt > 100.0f) {
            canvas_draw_str(canvas, 6, 49, "ZBLIZ (PIN > 100 ZL)");
        } else {
            canvas_draw_str(canvas, 6, 49, "PROSZE ZBLIZYC KARTE");
        }

        // Fale zbliżeniowe
        canvas_draw_disc(canvas, 112, 33, 2);
        canvas_draw_circle(canvas, 112, 33, 6);
        canvas_draw_circle(canvas, 112, 33, 10);
        canvas_draw_circle(canvas, 112, 33, 14);

        canvas_draw_line(canvas, 1, 51, 126, 51);
        canvas_draw_str(canvas, 6, 60, "Anuluj: Wstecz [Back]");
    }
    // ── 3. STAN: WPROWADZANIE KODU PIN NA EKRANIE FLIPPERA ────────────────────
    else if (app->state == FlipsonStatePinEntry) {
        canvas_set_font(canvas, FontSecondary);
        char pin_hdr[48];
        if (app->payment_amount[0] != '\0') {
            snprintf(pin_hdr, sizeof(pin_hdr), "%s PLN", app->payment_amount);
        } else {
            snprintf(pin_hdr, sizeof(pin_hdr), "KWOTA: --");
        }
        canvas_draw_str(canvas, 4, 23, pin_hdr);

        // 4 dyskretne, zaokrąglone kasetki na cyfry PIN (EMV PIN Pad)
        int box_start_x = 74;
        int box_y = 15;
        int box_w = 11;
        int box_h = 10;
        for (int i = 0; i < 4; i++) {
            int bx = box_start_x + (i * 13);
            canvas_draw_rframe(canvas, bx, box_y, box_w, box_h, 1);
            if (i < (int)app->entered_pin_len) {
                // Wypełniony dysk oznaczający wprowadzoną cyfrę
                canvas_draw_disc(canvas, bx + 5, box_y + 5, 2);
            } else if (i == (int)app->entered_pin_len) {
                // Delikatny kursor pod bieżącą pozycją
                canvas_draw_line(canvas, bx + 2, box_y + 8, bx + 8, box_y + 8);
            }
        }

        // Klawiatura ekranowa: 2 rzędy po 6 przycisków
        const char* key_labels[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "<", "OK"};
        int start_x = 3;
        int btn_w = 19;
        int btn_h = 10;

        for (int i = 0; i < 12; i++) {
            int row = (i < 6) ? 0 : 1;
            int col = (i < 6) ? i : (i - 6);
            int bx = start_x + (col * (btn_w + 2));
            int by = 28 + (row * (btn_h + 2));

            bool is_selected = (app->pin_cursor_pos == i);
            if (is_selected) {
                canvas_set_color(canvas, ColorBlack);
                canvas_draw_box(canvas, bx, by, btn_w, btn_h);
                canvas_set_color(canvas, ColorWhite);
            } else {
                canvas_set_color(canvas, ColorBlack);
                canvas_draw_frame(canvas, bx, by, btn_w, btn_h);
            }

            canvas_set_font(canvas, FontSecondary);
            int text_x = bx + ((btn_w - 7) / 2);
            if (i == 11) text_x = bx + 3; // "OK"
            canvas_draw_str(canvas, text_x, by + 8, key_labels[i]);
            canvas_set_color(canvas, ColorBlack);
        }

        canvas_set_font(canvas, FontSecondary);
        canvas_draw_line(canvas, 0, 52, 128, 52);
        canvas_draw_str(canvas, 4, 62, "Nawigacja: Strzalki | Wybierz: OK");
    }
    // ── 4. STAN: PŁATNOŚĆ PRZYJĘTA (SUKCES) ───────────────────────────────────
    else if (app->state == FlipsonStatePaymentApproved) {
        canvas_draw_rframe(canvas, 1, 14, 126, 49, 2);

        // Duża okrągła ikona [V] zatwierdzenia
        canvas_draw_circle(canvas, 18, 36, 12);
        canvas_draw_circle(canvas, 18, 36, 11);
        canvas_draw_line(canvas, 11, 36, 16, 41);
        canvas_draw_line(canvas, 16, 41, 25, 30);
        canvas_draw_line(canvas, 11, 37, 16, 42);
        canvas_draw_line(canvas, 16, 42, 25, 31);

        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 35, 26, "ZATWIERDZONO!");

        canvas_set_font(canvas, FontSecondary);
        char kwota_str[48];
        snprintf(kwota_str, sizeof(kwota_str), "Kwota: %s", app->payment_amount[0] ? app->payment_amount : "OK");
        canvas_draw_str(canvas, 35, 37, kwota_str);
        canvas_draw_str(canvas, 35, 47, "PROSZE ZABRAC KARTE");

        canvas_draw_line(canvas, 1, 51, 126, 51);
        canvas_draw_str(canvas, 6, 60, "DZIEKUJEMY ZA ZAKUPY!");
    }
    // ── 5. STAN: PŁATNOŚĆ ODRZUCONA (BŁĄD DLA KLIENTA) ────────────────────────
    else if (app->state == FlipsonStatePaymentDeclined) {
        canvas_draw_rframe(canvas, 1, 14, 126, 49, 2);

        // Duża okrągła ikona [X] odmowy
        canvas_draw_circle(canvas, 18, 36, 12);
        canvas_draw_circle(canvas, 18, 36, 11);
        canvas_draw_line(canvas, 12, 30, 24, 42);
        canvas_draw_line(canvas, 12, 42, 24, 30);
        canvas_draw_line(canvas, 13, 30, 25, 42);
        canvas_draw_line(canvas, 13, 42, 25, 30);

        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 35, 26, "ODRZUCONO!");

        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 35, 37, app->payment_msg[0] ? app->payment_msg : "Brak srodkow / Zly PIN");
        canvas_draw_str(canvas, 35, 47, "PROSZE ZABRAC KARTE");

        canvas_draw_line(canvas, 1, 51, 126, 51);
        canvas_draw_str(canvas, 6, 60, "Sprobuj inna metode");
    }
    // ── 6. STAN: PROGRAMATOR — ZBLIŻ CZYSTĄ KARTĘ ─────────────────────────────
    else if (app->state == FlipsonStateWriteWaiting) {
        canvas_draw_rframe(canvas, 1, 14, 126, 49, 2);

        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 26, "ZBLIZ CZYSTA KARTE");

        canvas_set_font(canvas, FontSecondary);
        if (strlen(app->write_req.pan) >= 12) {
            char mpan[24];
            size_t plen = strlen(app->write_req.pan);
            snprintf(mpan, sizeof(mpan), "PAN: %.4s **** %.4s", app->write_req.pan, &app->write_req.pan[plen - 4]);
            canvas_draw_str(canvas, 6, 38, mpan);
        } else {
            canvas_draw_str(canvas, 6, 38, "Programowanie sektorow");
        }
        canvas_draw_str(canvas, 6, 48, "Mifare Classic / NTAG");

        canvas_draw_line(canvas, 1, 51, 126, 51);
        canvas_draw_str(canvas, 6, 60, "Anuluj: Wstecz [Back]");
    }
    // ── 7. STAN: ZAPISANO POMYŚLNIE ───────────────────────────────────────────
    else if (app->state == FlipsonStateWriteSuccess) {
        canvas_draw_rframe(canvas, 1, 14, 126, 49, 2);

        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 27, "ZAPISANO POMYSLNIE!");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 6, 40, "Karta gotowa do platnosci");
        canvas_draw_str(canvas, 6, 49, "Mozna placic zblizeniowo");
        canvas_draw_line(canvas, 1, 51, 126, 51);
        canvas_draw_str(canvas, 6, 60, "Zdejmij karte z czytnika");
    }
    // ── 8. STAN: ODCZYT W TOKU ────────────────────────────────────────────────
    else if (app->state == FlipsonStateReading) {
        canvas_draw_rframe(canvas, 1, 14, 126, 49, 2);

        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 27, "ODCZYTYWANIE KARTY");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 6, 39, "NIE ZABIERAJ KARTY!");
        canvas_draw_str(canvas, 6, 49, "Autoryzacja w toku...");

        canvas_draw_frame(canvas, 6, 53, 116, 6);
        canvas_draw_box(canvas, 8, 55, 100, 2);
    }
    // ── 9. STAN: KARTA ODCZYTANA (SUKCES) ─────────────────────────────────────
    else if (app->state == FlipsonStateSuccess) {
        canvas_draw_rframe(canvas, 1, 14, 126, 49, 2);

        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 25, "KARTA ODCZYTANA!");

        canvas_set_font(canvas, FontSecondary);
        if (app->current_card.has_pan) {
            char masked[32];
            size_t plen = strlen(app->current_card.pan);
            snprintf(masked, sizeof(masked), "%.4s **** %.4s", app->current_card.pan, &app->current_card.pan[plen - 4]);
            canvas_draw_str(canvas, 6, 37, masked);
        } else {
            char uid_str[48];
            snprintf(uid_str, sizeof(uid_str), "UID: %s", app->current_card.uid_hex);
            canvas_draw_str(canvas, 6, 37, uid_str);
        }

        char proto_str[48];
        snprintf(proto_str, sizeof(proto_str), "TYP: %s", card_protocol_name(app->current_card.protocol));
        canvas_draw_str(canvas, 6, 47, proto_str);

        canvas_draw_line(canvas, 1, 51, 126, 51);
        canvas_draw_str(canvas, 6, 60, "Wyslano do kasy ->");
    }
    // ── 10. STAN: BŁĄD ────────────────────────────────────────────────────────
    else if (app->state == FlipsonStateError) {
        canvas_draw_rframe(canvas, 1, 14, 126, 49, 2);

        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 27, "BLAD OPERACJI!");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 6, 40, app->status_msg);
        canvas_draw_line(canvas, 1, 51, 126, 51);
        canvas_draw_str(canvas, 6, 60, "Sprobuj ponownie ->");
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
    FURI_LOG_I(TAG, "Starting FLIPSON POS Terminal Application v1.3");

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
    }

    FuriMessageQueue* event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, flipson_render_callback, app);
    view_port_input_callback_set(view_port, flipson_input_callback, event_queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    // Inicjalizacja NFC
    app->nfc = nfc_alloc();

    // Rejestracja komendy CLI 'flipson' w powłoce systemowej Flippera
    CliRegistry* cli = furi_record_open(RECORD_CLI);
    if (cli) {
        cli_registry_add_command(cli, "flipson", CliCommandFlagParallelSafe | CliCommandFlagInsomniaSafe, flipson_cli_callback, app);
        FURI_LOG_I(TAG, "Registered 'flipson' CLI command (ParallelSafe)");
    }

    // Start dedykowanego wątku roboczego NFC ze stosem 8KB
    app->worker_running = true;
    app->nfc_worker_thread = furi_thread_alloc_ex("FlipsonNfc", NFC_THREAD_STACK_SIZE, flipson_nfc_worker, app);
    furi_thread_start(app->nfc_worker_thread);

    app->running = true;
    InputEvent event;

    // Główna pętla zdarzeń interfejsu GUI i klawiatury
    while (app->running) {
        // Automatyczny powrót z ekranu sukcesu/odrzucenia po 3 sekundach
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        if (app->state == FlipsonStatePaymentApproved || app->state == FlipsonStatePaymentDeclined) {
            if ((furi_get_tick() - app->result_screen_tick) > furi_ms_to_ticks(4000)) {
                app->state = FlipsonStateWaiting;
                app->payment_amount[0] = '\0';
                app->payment_msg[0] = '\0';
                snprintf(app->status_msg, sizeof(app->status_msg), "Gotowy");
            }
        }
        furi_mutex_release(app->mutex);

        if (furi_message_queue_get(event_queue, &event, 50) == FuriStatusOk) {
            if (event.type == InputTypeShort || event.type == InputTypeRepeat) {
                furi_mutex_acquire(app->mutex, FuriWaitForever);
                FlipsonState st = app->state;
                furi_mutex_release(app->mutex);

                // ── OBSŁUGA KLAWIATURY W TRYBIE PODAWANIA PINU ─────────────────
                if (st == FlipsonStatePinEntry) {
                    if (event.key == InputKeyLeft) {
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        if (app->pin_cursor_pos > 0) app->pin_cursor_pos--;
                        else app->pin_cursor_pos = 11;
                        furi_mutex_release(app->mutex);
                        flipson_play_key_click();
                    } else if (event.key == InputKeyRight) {
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        if (app->pin_cursor_pos < 11) app->pin_cursor_pos++;
                        else app->pin_cursor_pos = 0;
                        furi_mutex_release(app->mutex);
                        flipson_play_key_click();
                    } else if (event.key == InputKeyUp) {
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        if (app->pin_cursor_pos >= 6) app->pin_cursor_pos -= 6;
                        furi_mutex_release(app->mutex);
                        flipson_play_key_click();
                    } else if (event.key == InputKeyDown) {
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        if (app->pin_cursor_pos < 6) app->pin_cursor_pos += 6;
                        furi_mutex_release(app->mutex);
                        flipson_play_key_click();
                    } else if (event.key == InputKeyOk) {
                        // Klawisz OK: wybierz cyfrę, Backspace (<) lub Zatwierdź (OK)
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        int pos = app->pin_cursor_pos;
                        bool submit_now = false;
                        if (pos >= 0 && pos <= 8) {
                            // Cyfry '1' do '9'
                            if (app->entered_pin_len < 4) {
                                app->entered_pin[app->entered_pin_len++] = '1' + pos;
                                app->entered_pin[app->entered_pin_len] = '\0';
                            }
                            flipson_play_key_click();
                            if (app->entered_pin_len == 4) {
                                app->pin_cursor_pos = 11;
                            }
                        } else if (pos == 9) {
                            // Cyfra '0'
                            if (app->entered_pin_len < 4) {
                                app->entered_pin[app->entered_pin_len++] = '0';
                                app->entered_pin[app->entered_pin_len] = '\0';
                            }
                            flipson_play_key_click();
                            if (app->entered_pin_len == 4) {
                                app->pin_cursor_pos = 11;
                            }
                        } else if (pos == 10) {
                            // Backspace (<)
                            if (app->entered_pin_len > 0) {
                                app->entered_pin_len--;
                                app->entered_pin[app->entered_pin_len] = '\0';
                            }
                            flipson_play_key_click();
                        } else if (pos == 11) {
                            // Zatwierdzenie PIN-u (OK)
                            if (app->entered_pin_len >= 4) {
                                submit_now = true;
                            } else {
                                flipson_play_error_buzz();
                            }
                        }
                        furi_mutex_release(app->mutex);

                        if (submit_now) {
                            flipson_transmit_pin_submit(app);
                            flipson_play_pos_chime();
                            furi_mutex_acquire(app->mutex, FuriWaitForever);
                            app->state = FlipsonStateReading;
                            snprintf(app->status_msg, sizeof(app->status_msg), "Weryfikacja PIN...");
                            furi_mutex_release(app->mutex);
                        }
                    } else if (event.key == InputKeyBack) {
                        // Anulowanie PIN-u
                        flipson_transmit_pin_cancel(app);
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        if (strlen(app->payment_amount) > 0) {
                            app->state = FlipsonStatePaymentPending;
                        } else {
                            app->state = FlipsonStateWaiting;
                        }
                        furi_mutex_release(app->mutex);
                    }
                }
                // ── OBSŁUGA POZOSTAŁYCH STANÓW ────────────────────────────────
                else if (event.key == InputKeyBack) {
                    if (st == FlipsonStateWriteWaiting || st == FlipsonStatePaymentPending) {
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        app->state = FlipsonStateWaiting;
                        app->write_req.active = false;
                        app->payment_amount[0] = '\0';
                        snprintf(app->status_msg, sizeof(app->status_msg), "Gotowy");
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

    FURI_LOG_I(TAG, "FLIPSON POS Terminal Exited cleanly");
    return 0;
}
