/**
 * FLIPSON POS Reader — Dedykowana Aplikacja Flipper Zero
 * Bezpośredni odczyt kart zbliżeniowych EMV (PAN 16 cyfr, data, holder)
 * oraz tagów zbliżeniowych i przesyłanie czystego JSON przez USB Serial (CDC VCP) i UART do kasy POS.
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
#include <nfc/protocols/iso14443_4a/iso14443_4a_poller.h>
#include <nfc/protocols/iso14443_4a/iso14443_4a.h>
#include <toolbox/bit_buffer.h>

#include "flipson_emv.h"

#define TAG "FlipsonPos"

typedef enum {
    FlipsonStateWaiting,
    FlipsonStateReading,
    FlipsonStateSuccess,
    FlipsonStateError
} FlipsonState;

typedef struct {
    FuriMutex* mutex;
    FlipsonState state;
    FlipsonCardData current_card;
    uint32_t tap_count;
    uint32_t last_tap_tick;
    bool usb_connected;
    char status_msg[48];
    FuriHalSerialHandle* serial_handle;
    Nfc* nfc;
    NfcPoller* poller;
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

// Bezpośrednie wysłanie JSON przez USB Serial (CDC VCP) oraz pin UART (13 TX, 115200 baud)
static void flipson_transmit_card_json(FlipsonAppContext* app, const FlipsonCardData* card) {
    uint32_t now_ts = furi_hal_rtc_get_timestamp();

    // Maskowany numer do logów
    char masked_pan[32] = "BRAK";
    if (card->has_pan && strlen(card->pan) >= 12) {
        size_t plen = strlen(card->pan);
        snprintf(masked_pan, sizeof(masked_pan), "%.4s********%.4s", card->pan, &card->pan[plen - 4]);
    }

    char json_buffer[512];
    snprintf(
        json_buffer,
        sizeof(json_buffer),
        "{\"event\":\"CARD_TAP\",\"source\":\"FLIPPER_ZERO\",\"device\":\"FLIPPER_POS_1.2\","
        "\"protocol\":\"%s\",\"uid\":\"%s\",\"card_number\":\"%s\",\"exp_month\":\"%s\","
        "\"exp_year\":\"%s\",\"cardholder\":\"%s\",\"token\":\"%s\",\"ts\":%lu}\r\n",
        card_protocol_name(card->protocol),
        card->uid_hex,
        card->has_pan ? card->pan : "",
        card->exp_month,
        card->exp_year,
        card->cardholder,
        card->secure_token,
        (unsigned long)now_ts
    );

    size_t len = strlen(json_buffer);

    // 1. Wysyłka przez USB VCP (Virtual COM Port do Electron POS / navigator.serial)
    furi_thread_stdout_write(json_buffer, len);

    // 2. Wysyłka przez port sprzętowy UART GPIO (Pin 13 TX, 115200 baud)
    if (app && app->serial_handle) {
        furi_hal_serial_tx(app->serial_handle, (const uint8_t*)json_buffer, len);
        furi_hal_serial_tx_wait_complete(app->serial_handle);
    }

    FURI_LOG_I(TAG, "Transmitted Card Tap JSON: UID=%s, PAN=%s", card->uid_hex, masked_pan);
}

// ── OBSŁUGA ISO14443-4 APDU (EMV) ─────────────────────────────────────────────
// APDU 1: SELECT PPSE ("2PAY.SYS.DDF01")
static const uint8_t APDU_SELECT_PPSE[] = {
    0x00, 0xA4, 0x04, 0x00, 0x0E,
    '2', 'P', 'A', 'Y', '.', 'S', 'Y', 'S', '.', 'D', 'D', 'F', '0', '1',
    0x00
};

// APDU 2: SELECT VISA AID (A0 00 00 00 03 10 10)
static const uint8_t APDU_SELECT_VISA[] = {
    0x00, 0xA4, 0x04, 0x00, 0x07,
    0xA0, 0x00, 0x00, 0x00, 0x03, 0x10, 0x10,
    0x00
};

// APDU 3: SELECT MASTERCARD AID (A0 00 00 00 04 10 10)
static const uint8_t APDU_SELECT_MC[] = {
    0x00, 0xA4, 0x04, 0x00, 0x07,
    0xA0, 0x00, 0x00, 0x00, 0x04, 0x10, 0x10,
    0x00
};

// APDU 4: READ RECORD (SFI 1, Record 1)
static const uint8_t APDU_READ_RECORD_1[] = {
    0x00, 0xB2, 0x01, 0x0C, 0x00
};

// APDU 5: READ RECORD (SFI 2, Record 1)
static const uint8_t APDU_READ_RECORD_2[] = {
    0x00, 0xB2, 0x01, 0x14, 0x00
};

// Bezpieczna wymiana bloków APDU z kartą ISO14443-4A za pomocą BitBuffer
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

// Próba odczytu rekordów EMV
static void flipson_emv_read_records(Iso14443_4aPoller* iso_poller, FlipsonCardData* card) {
    uint8_t rx_buf[256];
    size_t rx_len = 0;

    // Krok 1: Próba odczytu rekordu SFI 1, Rec 1
    if (flipson_apdu_exchange(iso_poller, APDU_READ_RECORD_1, sizeof(APDU_READ_RECORD_1), rx_buf, sizeof(rx_buf), &rx_len)) {
        if (rx_len > 2) {
            emv_parse_record_tlv(rx_buf, rx_len - 2, card);
        }
    }

    // Krok 2: Jeśli brak PAN, spróbuj SFI 2, Rec 1
    if (!card->has_pan) {
        if (flipson_apdu_exchange(iso_poller, APDU_READ_RECORD_2, sizeof(APDU_READ_RECORD_2), rx_buf, sizeof(rx_buf), &rx_len)) {
            if (rx_len > 2) {
                emv_parse_record_tlv(rx_buf, rx_len - 2, card);
            }
        }
    }
}

// Callback pollera NFC ISO14443-4A
static NfcCommand flipson_nfc_callback(NfcGenericEvent event, void* context) {
    FlipsonAppContext* app = (FlipsonAppContext*)context;
    if (!app) return NfcCommandStop;

    if (event.protocol == NfcProtocolIso14443_4a) {
        Iso14443_4aPollerEvent* iso_event = (Iso14443_4aPollerEvent*)event.event_data;
        if (!iso_event) return NfcCommandContinue;

        if (iso_event->type == Iso14443_4aPollerEventTypeReady) {
            Iso14443_4aPoller* iso_poller = (Iso14443_4aPoller*)event.instance;

            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->state = FlipsonStateReading;
            snprintf(app->status_msg, sizeof(app->status_msg), "Wykryto karte, czytam...");
            furi_mutex_release(app->mutex);

            FlipsonCardData detected_card;
            memset(&detected_card, 0, sizeof(detected_card));
            detected_card.protocol = CARD_PROTO_EMV_GENERIC;

            // Pobierz dane karty ISO14443-4A (w tym unikalny UID)
            const Iso14443_4aData* iso_data = (const Iso14443_4aData*)nfc_poller_get_data(app->poller);
            if (iso_data) {
                size_t uid_len = 0;
                const uint8_t* uid_ptr = iso14443_4a_get_uid(iso_data, &uid_len);
                if (uid_ptr && uid_len > 0) {
                    detected_card.uid_len = (uid_len > MAX_UID_LEN) ? MAX_UID_LEN : (uint8_t)uid_len;
                    memcpy(detected_card.uid_raw, uid_ptr, detected_card.uid_len);
                    for (size_t i = 0; i < detected_card.uid_len; i++) {
                        snprintf(&detected_card.uid_hex[i * 2], 3, "%02X", detected_card.uid_raw[i]);
                    }
                }
            }

            uint8_t rx_buf[256];
            size_t rx_len = 0;

            // Krok 1: SELECT PPSE ("2PAY.SYS.DDF01")
            if (flipson_apdu_exchange(iso_poller, APDU_SELECT_PPSE, sizeof(APDU_SELECT_PPSE), rx_buf, sizeof(rx_buf), &rx_len)) {
                if (rx_len > 2) {
                    emv_parse_record_tlv(rx_buf, rx_len - 2, &detected_card);
                }
            }

            // Krok 2: Jeśli nie ma PAN, spróbuj bezpośrednio VISA AID
            if (!detected_card.has_pan) {
                if (flipson_apdu_exchange(iso_poller, APDU_SELECT_VISA, sizeof(APDU_SELECT_VISA), rx_buf, sizeof(rx_buf), &rx_len)) {
                    detected_card.protocol = CARD_PROTO_EMV_VISA;
                    flipson_emv_read_records(iso_poller, &detected_card);
                }
            }

            // Krok 3: Jeśli nadal nie ma, spróbuj MASTERCARD AID
            if (!detected_card.has_pan) {
                if (flipson_apdu_exchange(iso_poller, APDU_SELECT_MC, sizeof(APDU_SELECT_MC), rx_buf, sizeof(rx_buf), &rx_len)) {
                    detected_card.protocol = CARD_PROTO_EMV_MASTERCARD;
                    flipson_emv_read_records(iso_poller, &detected_card);
                }
            }

            // Bezpieczny token identyfikacyjny karty (UID / EMV Token)
            if (!detected_card.has_token && strlen(detected_card.uid_hex) > 0) {
                snprintf(detected_card.secure_token, sizeof(detected_card.secure_token), "EMV_SEC_%s", detected_card.uid_hex);
                detected_card.has_token = true;
            }

            // Zapisz wynik i zaktualizuj stan
            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->current_card = detected_card;
            app->tap_count++;
            app->last_tap_tick = furi_get_tick();
            app->state = FlipsonStateSuccess;
            snprintf(app->status_msg, sizeof(app->status_msg), "Wyslano do POS!");
            furi_mutex_release(app->mutex);

            // Sygnalizuj sukces (USB VCP + UART Pin 13 + dźwięk terminala)
            flipson_transmit_card_json(app, &detected_card);
            flipson_play_pos_chime();

            furi_delay_ms(1500);

            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->state = FlipsonStateWaiting;
            snprintf(app->status_msg, sizeof(app->status_msg), "Zbliz karte do Flippera");
            furi_mutex_release(app->mutex);

            return NfcCommandContinue;
        }
    }

    return NfcCommandContinue;
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
    canvas_draw_str(canvas, 76, 9, "USB & UART");

    canvas_set_color(canvas, ColorBlack);

    if (app->state == FlipsonStateWaiting) {
        // Ekran oczekiwania na zbliżenie
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 6, 26, "ZBLIZ KARTE NFC");

        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 6, 38, "EMV / Visa / Mastercard");
        canvas_draw_str(canvas, 6, 48, "lub Apple / Google Pay");

        // Dolny pasek
        canvas_draw_line(canvas, 0, 52, 128, 52);
        char stats_str[48];
        snprintf(stats_str, sizeof(stats_str), "Odczytow: %lu | OK", (unsigned long)app->tap_count);
        canvas_draw_str(canvas, 4, 62, stats_str);

        // Rysuj ikonkę fali radiowej
        canvas_draw_circle(canvas, 115, 30, 4);
        canvas_draw_circle(canvas, 115, 30, 8);
        canvas_draw_circle(canvas, 115, 30, 12);

    } else if (app->state == FlipsonStateReading) {
        // Ekran czytania
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 10, 30, "ODCZYT APDU...");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 10, 45, "Trzymaj przy pleckach!");

    } else if (app->state == FlipsonStateSuccess) {
        // Ekran sukcesu
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
        canvas_draw_str(canvas, 4, 62, "Gotowy na nastepna ->");
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
    FURI_LOG_I(TAG, "Starting FLIPSON POS Reader App");

    FlipsonAppContext* app = malloc(sizeof(FlipsonAppContext));
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

    // Inicjalizacja NFC (ISO 14443-4A dla kart płatniczych EMV / kart z mikrokontrolerem)
    app->nfc = nfc_alloc();
    app->poller = nfc_poller_alloc(app->nfc, NfcProtocolIso14443_4a);
    nfc_poller_start(app->poller, flipson_nfc_callback, app);

    InputEvent event;
    bool running = true;
    while (running) {
        if (furi_message_queue_get(event_queue, &event, 100) == FuriStatusOk) {
            if (event.type == InputTypeShort && event.key == InputKeyBack) {
                running = false;
            }
        }
        view_port_update(view_port);
    }

    // Sprzątanie zasobów NFC
    nfc_poller_stop(app->poller);
    nfc_poller_free(app->poller);
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
