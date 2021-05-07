#pragma once
#include <stdint.h>
using uint = unsigned int;

void scrambleReverseInput(uint8_t* buff, uint size);
void scrambleReverseOutput(uint8_t* buff, uint size);
uint8_t reverseBits(uint8_t b);
