#include <iostream>
#include <stdint.h>
#include <stdio.h>

#define IMAGE_ROWS 120

void filterImage(volatile uint8_t rowBelow[IMAGE_ROWS], volatile uint8_t rowCenter[IMAGE_ROWS], volatile uint8_t rowAbove[IMAGE_ROWS], volatile uint8_t outputRow[IMAGE_ROWS]);
