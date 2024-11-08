
#include "image_filter.hpp"

const uint8_t kernel_center = 16;    // 0.25 * 64
const uint8_t kernel_adjacent = 8;   // 0.125 * 64
const uint8_t kernel_diagonal = 4;   // 0.0625 * 64

void filterImage(volatile uint8_t rowBelow[IMAGE_ROWS],
                 volatile uint8_t rowCenter[IMAGE_ROWS],
                 volatile uint8_t rowAbove[IMAGE_ROWS],
                 volatile uint8_t outputRow[IMAGE_ROWS])
{

#pragma HLS INTERFACE s_axilite port=return bundle=AXI_CPU
#pragma HLS INTERFACE s_axilite port=rowBelow bundle=AXI_CPU
#pragma HLS INTERFACE s_axilite port=rowCenter bundle=AXI_CPU
#pragma HLS INTERFACE s_axilite port=rowAbove bundle=AXI_CPU
#pragma HLS INTERFACE s_axilite port=outputRow bundle=AXI_CPU

	for (int x = 1; x < IMAGE_ROWS - 1; ++x) {
		#pragma HLS UNROLL

		// Calculate the convolution using fixed-point arithmetic (scaled by 64)
		uint32_t pixelValue = 0;
		pixelValue += rowCenter[x] * kernel_center;

		uint16_t sum_adjacent = rowAbove[x] + rowBelow[x] + rowCenter[x - 1] + rowCenter[x + 1];
		pixelValue += sum_adjacent * kernel_adjacent;

		uint16_t sum_diagonal = rowAbove[x - 1] + rowAbove[x + 1] + rowBelow[x - 1] + rowBelow[x + 1];
		pixelValue += sum_diagonal * kernel_diagonal;

		// Scale back to 8-bit range and clamp
		pixelValue = (pixelValue + 32) >> 6; // Equivalent to dividing by 64 with rounding
		outputRow[x] = (uint8_t)(std::min(pixelValue, (uint32_t)255));
	}

	// Set boundary pixels to zero
	outputRow[0] = 0;
	outputRow[IMAGE_ROWS - 1] = 0;
}
