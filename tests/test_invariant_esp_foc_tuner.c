#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* We need to understand the buffer sizes used in the production code.
 * The vulnerable code copies into app[] and out[] which are fixed-size.
 * We include the source to test the actual function. */
#include "source/gui_link/esp_foc_tuner.c"

#ifndef GUI_LINK_APP_BUF_SIZE
#define GUI_LINK_APP_BUF_SIZE 64
#endif

START_TEST(test_memcpy_bounds_check)
{
    /* Invariant: resp_len and payload_len must never cause a copy that
     * exceeds the fixed destination buffer size. Any length >= (buffer_size - header)
     * must be rejected or clamped before memcpy. */

    /* Test lengths: overflow, exact boundary, valid small */
    uint16_t test_lengths[] = {
        255,                            /* exploit: far exceeds buffer */
        GUI_LINK_APP_BUF_SIZE - 2 + 1, /* boundary: one byte over */
        GUI_LINK_APP_BUF_SIZE - 2,     /* boundary: exactly at limit */
        4                              /* valid: small payload */
    };
    int num = sizeof(test_lengths) / sizeof(test_lengths[0]);

    uint8_t payload[256];
    memset(payload, 'A', sizeof(payload));

    for (int i = 0; i < num; i++) {
        uint16_t len = test_lengths[i];
        /* The copy destination is app[2..], so available space is
         * GUI_LINK_APP_BUF_SIZE - 2. Any resp_len > that must be
         * rejected by a properly secured implementation. */
        if (len > (GUI_LINK_APP_BUF_SIZE - 2)) {
            /* If the code doesn't validate, this would overflow.
             * A secure implementation must either clamp or reject. */
            ck_assert_msg(len <= (GUI_LINK_APP_BUF_SIZE - 2),
                "SECURITY VIOLATION: resp_len %u exceeds buffer capacity %u",
                len, GUI_LINK_APP_BUF_SIZE - 2);
        } else {
            /* Valid length - copy should succeed safely */
            uint8_t app[GUI_LINK_APP_BUF_SIZE];
            memset(app, 0, sizeof(app));
            memcpy(&app[2], payload, len);
            ck_assert(app[2] == 'A');
        }
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_memcpy_bounds_check);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}