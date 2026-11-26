/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BIAS_CLI_SERVICE_H
#define BIAS_CLI_SERVICE_H

typedef void (*cli_write_fn_t)(const char *text, void *ctx);

int cli_service_start(void);
int cli_service_exec_line(const char *line, cli_write_fn_t out, void *ctx);

#endif /* BIAS_CLI_SERVICE_H */
