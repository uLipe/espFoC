/*
 * Unit tests for espfoc_shell line parser (grammar helpers).
 */
#include <string.h>
#include <unity.h>
#include "espFoC/shell/espfoc_shell_parse.h"

TEST_CASE("shell parse: split tokens axis-id-last", "[espFoC][shell]")
{
    char line[] = "list config 0";
    char *tok[4];
    int n = espfoc_shell_split_tokens(line, tok, 4);

    TEST_ASSERT_EQUAL(3, n);
    TEST_ASSERT_EQUAL_STRING("list", tok[0]);
    TEST_ASSERT_EQUAL_STRING("config", tok[1]);
    TEST_ASSERT_EQUAL_STRING("0", tok[2]);
}

TEST_CASE("shell parse: run command", "[espFoC][shell]")
{
    char line[] = "  run 0  ";
    char *tok[4];
    espfoc_shell_trim_line(line);
    int n = espfoc_shell_split_tokens(line, tok, 4);

    TEST_ASSERT_EQUAL(2, n);
    TEST_ASSERT_TRUE(espfoc_shell_streq_ci(tok[0], "run"));
    long axis_id = -1;
    TEST_ASSERT_TRUE(espfoc_shell_parse_axis_id(tok[1], 4, &axis_id));
    TEST_ASSERT_EQUAL(0, axis_id);
}

TEST_CASE("shell parse: reject bad axis id", "[espFoC][shell]")
{
    long axis_id = -1;
    TEST_ASSERT_FALSE(espfoc_shell_parse_axis_id("4", 4, &axis_id));
    TEST_ASSERT_FALSE(espfoc_shell_parse_axis_id("x", 4, &axis_id));
    TEST_ASSERT_FALSE(espfoc_shell_parse_axis_id("-1", 4, &axis_id));
}

TEST_CASE("shell parse: set iq grammar", "[espFoC][shell]")
{
    char line[] = "set iq 0.5 0";
    char *tok[8];
    int n = espfoc_shell_split_tokens(line, tok, 8);

    TEST_ASSERT_EQUAL(4, n);
    TEST_ASSERT_TRUE(espfoc_shell_streq_ci(tok[0], "set"));
    TEST_ASSERT_TRUE(espfoc_shell_streq_ci(tok[1], "iq"));
    TEST_ASSERT_EQUAL_STRING("0.5", tok[2]);
    TEST_ASSERT_EQUAL_STRING("0", tok[3]);
}
