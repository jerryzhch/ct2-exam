#include <stdio.h>
#include <reg_stm32f4xx.h>
#include <hal_adc.h>

#define printAddr(x) printf("addr of %s is %p\n", #x, &x);

int main() {
    printAddr(TIM8->CCR1);
    printAddr(ADC1->CR1);

    return 0;
}