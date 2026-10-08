/*
 * Auto-run Unity suite for greenfield espFoC (no interactive menu).
 * Build: idf.py -D TEST_COMPONENTS=espFoC set-target esp32c6 build
 * Host:  idf.py --preview -D TEST_COMPONENTS=espFoC set-target linux build
 */
#include <stdio.h>
#include <stdlib.h>

#include "sdkconfig.h"
#include "unity.h"
#include "unity_test_runner.h"

void app_main(void)
{
    printf("espFoC unit_test_runner: start\n");
    UNITY_BEGIN();
#if CONFIG_IDF_TARGET_LINUX
    unity_run_tests_by_tag("[known_fail]", true);
#else
    unity_run_all_tests();
#endif
    int fails = UNITY_END();
    if (fails == 0) {
        printf("PASS unit_tests\n");
    } else {
        printf("FAIL unit_tests fails=%d\n", fails);
    }
#if CONFIG_IDF_TARGET_LINUX
    exit(fails == 0 ? EXIT_SUCCESS : EXIT_FAILURE);
#endif
}
