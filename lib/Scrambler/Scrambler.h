#pragma once
#include <stdint.h>
using uint = unsigned int;

bool isSyncFrame(uint8_t* buff);
void scrambleReverseInput(uint8_t* buff, uint size);
void scrambleReverseOutput(uint8_t* buff, uint size);
