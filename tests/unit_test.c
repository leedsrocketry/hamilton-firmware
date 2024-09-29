#ifdef TEST

#include "../src/HAL/STM32_init.h"
#include "../src/frame_buffer.h"
#include <stdio.h>
#include <stdint.h>
#include "unity/unity.h"

// These always need to be defined for firmware to run
volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

// Unity requires these to be defined, would typically be done in standard library
// whic we do not use.
void *__exidx_start = 0;
void *__exidx_end = 0;
void abort(void) {
    while(1) { 
        __asm__("nop");
    }
}

// Setup and teardown functions for Unity
void setUp(){}
void tearDown(){}

// Framebuffer test
void test_framebuffer(void)
{
    // test get_median
    double test_data[] = {1.0, 3.0, 2.0};
    uint32_t test_data_size = 3;
    TEST_ASSERT_EQUAL_FLOAT(get_median(test_data, test_data_size), 2.0);

    // test get_framebuffer_median
    // Setup FrameBuffer
    FrameBuffer fb;
    init_frame_buffer(&fb);

    get_framebuffer_median(&fb, BUFFER_SIZE, MS5611_PRESSURE)
}


int main() {
    STM32_init();
    UNITY_BEGIN();
    printf("RUNNING TESTS: \r\n");

    RUN_TEST(test_framebuffer);

    printf("All tests completed.\r\n");
    return UNITY_END();
}

#endif
