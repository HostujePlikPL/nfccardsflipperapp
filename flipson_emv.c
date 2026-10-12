#include "flipson_emv.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

const char* card_protocol_name(CardProtoType proto) {
    switch (proto) {
        case CARD_PROTO_EMV_VISA: return "EMV_VISA";
        case CARD_PROTO_EMV_MASTERCARD: return "EMV_MASTERCARD";
        case CARD_PROTO_EMV_GENERIC: return "EMV_CONTACTLESS";
        case CARD_PROTO_MIFARE_CLASSIC: return "MIFARE_CLASSIC";
        case CARD_PROTO_MIFARE_ULTRALIGHT: return "MIFARE_ULTRALIGHT";
        case CARD_PROTO_NFC_TAG: return "ISO14443_A_TAG";
        default: return "UNKNOWN";
    }
}

void emv_bcd_to_ascii(const uint8_t* bcd, size_t bcd_len, char* out, size_t max_out) {
    size_t out_idx = 0;
    for (size_t i = 0; i < bcd_len && (out_idx + 2) < max_out; i++) {
        uint8_t high = (bcd[i] >> 4) & 0x0F;
        uint8_t low = bcd[i] & 0x0F;

        if (high <= 9) {
            out[out_idx++] = '0' + high;
        } else if (high == 0x0D) {
            out[out_idx++] = '='; // Separator w Track 2
        } else if (high == 0x0F) {
            break; // Padding
        }

        if (low <= 9) {
            out[out_idx++] = '0' + low;
        } else if (low == 0x0D) {
            out[out_idx++] = '='; // Separator w Track 2
        } else if (low == 0x0F) {
            break; // Padding
        }
    }
    out[out_idx] = '\0';
}

static void parse_track2(const uint8_t* val, size_t len, FlipsonCardData* card) {
    char t2[64];
    emv_bcd_to_ascii(val, len, t2, sizeof(t2));

    // Format Track 2: [PAN] = / D [YYMM] [ServiceCode] ...
    char* sep = strchr(t2, '=');
    if (!sep) sep = strchr(t2, 'D');

    if (sep) {
        size_t pan_len = sep - t2;
        if (pan_len >= 12 && pan_len <= 19 && !card->has_pan) {
            strncpy(card->pan, t2, pan_len);
            card->pan[pan_len] = '\0';
            card->has_pan = true;
        }

        // Data ważności zaraz po separatorze: YYMM
        if (strlen(sep + 1) >= 4) {
            char yy[3] = { sep[1], sep[2], '\0' };
            char mm[3] = { sep[3], sep[4], '\0' };
            snprintf(card->exp_year, sizeof(card->exp_year), "20%s", yy);
            snprintf(card->exp_month, sizeof(card->exp_month), "%s", mm);
        }
    }
}

bool emv_parse_record_tlv(const uint8_t* data, size_t len, FlipsonCardData* card) {
    if (!data || len == 0 || !card) return false;

    size_t i = 0;
    bool found_any = false;

    while (i < len) {
        // Pomiń bajty wypełnienia 0x00 lub 0xFF
        if (data[i] == 0x00 || data[i] == 0xFF) {
            i++;
            continue;
        }

        uint32_t tag = data[i++];
        if ((tag & 0x1F) == 0x1F) {
            // Tag dwubajtowy
            if (i >= len) break;
            tag = (tag << 8) | data[i++];
        }

        if (i >= len) break;

        // Długość wartości TLV
        size_t vlen = data[i++];
        if (vlen & 0x80) {
            uint8_t count = vlen & 0x7F;
            vlen = 0;
            while (count-- > 0 && i < len) {
                vlen = (vlen << 8) | data[i++];
            }
        }

        if (i + vlen > len) {
            // Uszkodzona ramka TLV
            break;
        }

        const uint8_t* val = &data[i];

        // Sprawdź tagi
        if (tag == 0x5A) {
            // Tag 5A: PAN (Numer karty w BCD)
            char pan_buf[32];
            emv_bcd_to_ascii(val, vlen, pan_buf, sizeof(pan_buf));
            // Usuń ewentualne 'F' na końcu
            size_t plen = strlen(pan_buf);
            if (plen >= 12 && plen <= 19) {
                strncpy(card->pan, pan_buf, sizeof(card->pan) - 1);
                card->has_pan = true;
                found_any = true;
            }
        } else if (tag == 0x57) {
            // Tag 57: Track 2 Equivalent Data
            parse_track2(val, vlen, card);
            found_any = true;
        } else if (tag == 0x5F24) {
            // Tag 5F24: Data ważności karty (YYMMDD BCD)
            if (vlen >= 3) {
                char exp_buf[10];
                emv_bcd_to_ascii(val, 3, exp_buf, sizeof(exp_buf));
                if (strlen(exp_buf) >= 4) {
                    snprintf(card->exp_year, sizeof(card->exp_year), "20%c%c", exp_buf[0], exp_buf[1]);
                    snprintf(card->exp_month, sizeof(card->exp_month), "%c%c", exp_buf[2], exp_buf[3]);
                    found_any = true;
                }
            }
        } else if (tag == 0x5F20) {
            // Tag 5F20: Cardholder Name (ASCII)
            size_t copy_len = vlen < (sizeof(card->cardholder) - 1) ? vlen : (sizeof(card->cardholder) - 1);
            memcpy(card->cardholder, val, copy_len);
            card->cardholder[copy_len] = '\0';
            found_any = true;
        } else if (tag == 0x70 || tag == 0x77 || tag == 0x61 || tag == 0xA5) {
            // Tag kontenerowy / szablon - rekurencyjne parsowanie wnętrza
            if (emv_parse_record_tlv(val, vlen, card)) {
                found_any = true;
            }
        }

        i += vlen;
    }

    return found_any;
}
