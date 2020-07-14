#ifndef _VSM_CLI_H_
#define _VSM_CLI_H_

#include <stdint.h>

void cli_input_insert(uint8_t *buf, uint32_t len);
void vsm_cli_start(void);

#endif /* _VSM_CLI_H_ */
