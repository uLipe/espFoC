/*
 * MIT License
 *
 * Copyright (c) 2021 Felipe Neves
 *
 * Shell line parser — testable subset of espfoc_shell grammar helpers.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void espfoc_shell_trim_line(char *s);

/** Tokenize @p line in-place (NUL-separators). Returns token count. */
int espfoc_shell_split_tokens(char *line, char **tok, int max_tok);

bool espfoc_shell_streq_ci(const char *a, const char *b);

/** Parse decimal axis id in [0, max_axis). */
bool espfoc_shell_parse_axis_id(const char *tok, long max_axis, long *axis_id_out);

#ifdef __cplusplus
}
#endif
