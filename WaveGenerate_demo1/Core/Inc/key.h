#ifndef __KEY_H
#define __KEY_H

#include "main.h"

struct key
{
	uint8_t sta;
	uint8_t last_sta;
	uint8_t short_flag;
};

void key_serv(void);
void key_proc(void);

#endif