#ifdef TEST

#include "../src/HAL/STM32_init.h"
#include "../src/frame_buffer.h"
#include <stdio.h>
#include <stdint.h>

// Setup custom assert
#undef assert
#define assert(expr) \
    if (!(expr)) { printf("TEST FAIL\r\n"); } else { printf("TEST PASS\r\n"); }

#define lura_test(name, expr) \
    if (!(expr)) { printf("%s: FAIL\r\n", name); } else { printf("%s: pass\r\n", name); }


volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

double test_data[] = {1.0, 3.0, 2.0};
uint32_t test_data_size = 3;

int main() {

    STM32_init();
    printf("RUNNING TESTS: \r\n");

    lura_test("test median", get_median(test_data, test_data_size) == 2.0);
    lura_test("test median 2", get_median(test_data, test_data_size) == 3.0);
    return 0;
}

#endif