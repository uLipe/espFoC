/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 */

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "espFoC/shell/espfoc_shell_parse.h"

void espfoc_shell_trim_line(char *s)
{
    if (s == NULL) {
        return;
    }
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r' ||
                    s[n - 1] == ' ' || s[n - 1] == '\t')) {
        s[--n] = '\0';
    }
}

int espfoc_shell_split_tokens(char *line, char **tok, int max_tok)
{
    int n = 0;
    char *p = line;

    if (line == NULL || tok == NULL || max_tok <= 0) {
        return 0;
    }

    while (*p && n < max_tok) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') {
            p++;
        }
        if (*p != '\0') {
            *p++ = '\0';
        }
    }
    return n;
}

bool espfoc_shell_streq_ci(const char *a, const char *b)
{
    if (a == NULL || b == NULL) {
        return false;
    }
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return false;
        }
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

bool espfoc_shell_parse_axis_id(const char *tok, long max_axis, long *axis_id_out)
{
    char *end = NULL;
    long id;

    if (tok == NULL || axis_id_out == NULL || max_axis <= 0) {
        return false;
    }
    id = strtol(tok, &end, 10);
    if (end == tok || *end != '\0' || id < 0 || id >= max_axis) {
        return false;
    }
    *axis_id_out = id;
    return true;
}
