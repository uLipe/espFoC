/*
 * MIT License
 *
 * Unit tests for esp_foc_link (wire framing + CRC). Pure C, no transport.
 */

#include <string.h>
#include <unity.h>
#include "espFoC/esp_foc_link.h"

static void encode_decode_roundtrip(esp_foc_link_channel_t channel,
                                    uint8_t seq,
                                    const uint8_t *payload, size_t len)
{
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t written = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK,
        esp_foc_link_encode(channel, seq, payload, len,
                            frame, sizeof(frame), &written));
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_HEADER_BYTES + len + ESP_FOC_LINK_TRAILER_BYTES,
                      written);

    esp_foc_link_decoder_t dec;
    esp_foc_link_decoder_reset(&dec);

    esp_foc_link_status_t st = ESP_FOC_LINK_ERR_NEED_MORE;
    for (size_t i = 0; i < written; ++i) {
        st = esp_foc_link_decoder_push(&dec, frame[i]);
        if (i + 1 < written) {
            TEST_ASSERT_EQUAL(ESP_FOC_LINK_ERR_NEED_MORE, st);
        }
    }
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK, st);
    TEST_ASSERT_EQUAL(channel, esp_foc_link_decoder_channel(&dec));
    TEST_ASSERT_EQUAL(seq, esp_foc_link_decoder_seq(&dec));
    TEST_ASSERT_EQUAL(len, esp_foc_link_decoder_payload_len(&dec));
    if (len > 0) {
        TEST_ASSERT_EQUAL_MEMORY(payload,
                                 esp_foc_link_decoder_payload(&dec), len);
    }
}

TEST_CASE("link: encode/decode round-trip on tiny payload",
          "[espFoC][link]")
{
    const uint8_t pl[] = {0xDE, 0xAD, 0xBE, 0xEF};
    encode_decode_roundtrip(ESP_FOC_LINK_CH_TUNER, 0x42, pl, sizeof(pl));
}

TEST_CASE("link: encode/decode round-trip on empty payload",
          "[espFoC][link]")
{
    encode_decode_roundtrip(ESP_FOC_LINK_CH_LOG, 0x00, NULL, 0);
}

TEST_CASE("link: encode/decode round-trip on max payload",
          "[espFoC][link]")
{
    uint8_t pl[ESP_FOC_LINK_MAX_PAYLOAD];
    for (size_t i = 0; i < sizeof(pl); ++i) {
        pl[i] = (uint8_t)(i ^ 0x55);
    }
    encode_decode_roundtrip(ESP_FOC_LINK_CH_SCOPE, 0xFF, pl, sizeof(pl));
}

TEST_CASE("link: encode rejects oversized payload",
          "[espFoC][link]")
{
    uint8_t out[ESP_FOC_LINK_MAX_FRAME];
    uint8_t pl[ESP_FOC_LINK_MAX_PAYLOAD + 1] = {0};
    size_t written = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_ERR_TOO_BIG,
        esp_foc_link_encode(ESP_FOC_LINK_CH_TUNER, 0, pl, sizeof(pl),
                            out, sizeof(out), &written));
}

TEST_CASE("link: encode rejects undersized output buffer",
          "[espFoC][link]")
{
    uint8_t out[4];
    uint8_t pl[1] = {0xAA};
    size_t written = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_ERR_TOO_BIG,
        esp_foc_link_encode(ESP_FOC_LINK_CH_TUNER, 0, pl, sizeof(pl),
                            out, sizeof(out), &written));
}

TEST_CASE("link: decoder skips garbage until SYNC byte",
          "[espFoC][link]")
{
    /* Pre-send a few bytes of "noise" before a valid frame. */
    const uint8_t pl[] = {0x10, 0x20};
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t written = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK,
        esp_foc_link_encode(ESP_FOC_LINK_CH_TUNER, 7, pl, sizeof(pl),
                            frame, sizeof(frame), &written));

    esp_foc_link_decoder_t dec;
    esp_foc_link_decoder_reset(&dec);

    /* Garbage bytes — none are SYNC, so decoder must stay in WAIT_SYNC. */
    const uint8_t garbage[] = {0x00, 0x11, 0xFF, 0x12, 0x34};
    for (size_t i = 0; i < sizeof(garbage); ++i) {
        TEST_ASSERT_EQUAL(ESP_FOC_LINK_ERR_NEED_MORE,
                          esp_foc_link_decoder_push(&dec, garbage[i]));
    }

    /* Now feed the real frame; should still parse correctly. */
    esp_foc_link_status_t st = ESP_FOC_LINK_ERR_NEED_MORE;
    for (size_t i = 0; i < written; ++i) {
        st = esp_foc_link_decoder_push(&dec, frame[i]);
    }
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK, st);
    TEST_ASSERT_EQUAL(7, esp_foc_link_decoder_seq(&dec));
}

TEST_CASE("link: decoder flags CRC corruption", "[espFoC][link]")
{
    const uint8_t pl[] = {0xCA, 0xFE};
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t written = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK,
        esp_foc_link_encode(ESP_FOC_LINK_CH_SCOPE, 1, pl, sizeof(pl),
                            frame, sizeof(frame), &written));

    /* Flip one CRC byte. */
    frame[written - 1] ^= 0xFF;

    esp_foc_link_decoder_t dec;
    esp_foc_link_decoder_reset(&dec);
    esp_foc_link_status_t st = ESP_FOC_LINK_ERR_NEED_MORE;
    for (size_t i = 0; i < written; ++i) {
        st = esp_foc_link_decoder_push(&dec, frame[i]);
    }
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_ERR_BAD_CRC, st);
}

TEST_CASE("link: decoder recovers after a corrupt frame",
          "[espFoC][link]")
{
    /* Bad frame followed by a good one: decoder must come back online. */
    const uint8_t pl[] = {0x01, 0x02, 0x03};
    uint8_t frame[ESP_FOC_LINK_MAX_FRAME];
    size_t written = 0;
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK,
        esp_foc_link_encode(ESP_FOC_LINK_CH_TUNER, 9, pl, sizeof(pl),
                            frame, sizeof(frame), &written));

    esp_foc_link_decoder_t dec;
    esp_foc_link_decoder_reset(&dec);

    /* Send a corrupt header (claims oversized payload). */
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_ERR_NEED_MORE,
                      esp_foc_link_decoder_push(&dec, ESP_FOC_LINK_SYNC));
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_ERR_NEED_MORE,
                      esp_foc_link_decoder_push(&dec, 0xFF));
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_ERR_TOO_BIG,
                      esp_foc_link_decoder_push(&dec, 0xFF));

    /* Decoder must be reset and now accept a valid frame. */
    esp_foc_link_status_t st = ESP_FOC_LINK_ERR_NEED_MORE;
    for (size_t i = 0; i < written; ++i) {
        st = esp_foc_link_decoder_push(&dec, frame[i]);
    }
    TEST_ASSERT_EQUAL(ESP_FOC_LINK_OK, st);
    TEST_ASSERT_EQUAL_MEMORY(pl, esp_foc_link_decoder_payload(&dec), sizeof(pl));
}

TEST_CASE("link: CRC-16/CCITT golden vectors", "[espFoC][link]")
{
    /* Reference values from independent computation. */
    const uint8_t empty[] = {0};
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, esp_foc_link_crc16(NULL, 0));
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, esp_foc_link_crc16(empty, 0));

    const uint8_t one[1] = {0x00};
    TEST_ASSERT_EQUAL_HEX16(0xE1F0u, esp_foc_link_crc16(one, 1));

    const uint8_t hello[5] = {'H','E','L','L','O'};
    /* Pre-computed in a separate Python session for cross-val. */
    TEST_ASSERT_EQUAL_HEX16(0x49D6u, esp_foc_link_crc16(hello, 5));
}
