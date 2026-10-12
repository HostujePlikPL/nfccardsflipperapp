#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MAX_PAN_LEN 24
#define MAX_NAME_LEN 36
#define MAX_UID_LEN 16
#define MAX_TOKEN_LEN 65

typedef enum {
    CARD_PROTO_UNKNOWN = 0,
    CARD_PROTO_EMV_VISA,
    CARD_PROTO_EMV_MASTERCARD,
    CARD_PROTO_EMV_GENERIC,
    CARD_PROTO_MIFARE_CLASSIC,
    CARD_PROTO_MIFARE_ULTRALIGHT,
    CARD_PROTO_NFC_TAG
} CardProtoType;

typedef struct {
    CardProtoType protocol;
    char uid_hex[MAX_UID_LEN * 2 + 1];
    uint8_t uid_raw[MAX_UID_LEN];
    uint8_t uid_len;
    char pan[MAX_PAN_LEN];              // 16-19 cyfrowy numer karty płatniczej
    char exp_month[4];                  // MM (np. 12)
    char exp_year[6];                   // YYYY (np. 2028)
    char cardholder[MAX_NAME_LEN];      // Imię i nazwisko posiadacza (jeśli zapisane na karcie)
    char secure_token[MAX_TOKEN_LEN];   // Token z bezpiecznego sektora karty
    bool has_pan;
    bool has_token;
} FlipsonCardData;

// Funkcje pomocnicze parsera EMV TLV oraz dekodera BCD
bool emv_parse_record_tlv(const uint8_t* data, size_t len, FlipsonCardData* card);
void emv_bcd_to_ascii(const uint8_t* bcd, size_t bcd_len, char* out, size_t max_out);
const char* card_protocol_name(CardProtoType proto);
