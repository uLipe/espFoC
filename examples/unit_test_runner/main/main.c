/*
 * Unit test runner for espFoC.
 * app_main only starts the Unity test menu; all test cases are in the espFoC test component.
 *
 * Build: idf.py -D TEST_COMPONENTS=espFoC build
 * Run:   idf.py flash monitor, then press Enter and type * to run all tests.
 */
#include "unity.h"

void app_main(void)
{
    unity_run_menu();
}
