#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>

#include "mraa.h"

#define thumb_f 8
#define index_f 9
#define middle_f 10
#define ring_f 11
#define pinky_f 12

int running = 0;

int
main(int argc, char** argv)
{
    mraa_result_t r = MRAA_SUCCESS;

    mraa_init();
    fprintf(stdout, "MRAA Version: %s\n", mraa_get_version());

    mraa_gpio_context gpio[5];
    gpio[0] = mraa_gpio_init(thumb_f);
    gpio[1] = mraa_gpio_init(index_f);
    gpio[2] = mraa_gpio_init(middle_f);
    gpio[3] = mraa_gpio_init(ring_f);
    gpio[4] = mraa_gpio_init(pinky_f);



    // set direction to OUT
    r = mraa_gpio_dir(gpio[0], MRAA_GPIO_OUT);
    r = mraa_gpio_dir(gpio[1], MRAA_GPIO_OUT);
    r = mraa_gpio_dir(gpio[2], MRAA_GPIO_OUT);
    r = mraa_gpio_dir(gpio[3], MRAA_GPIO_OUT);
    r = mraa_gpio_dir(gpio[4], MRAA_GPIO_OUT);



    while (running == 0) {
       mraa_gpio_write(gpio[0], 0);
       mraa_gpio_write(gpio[1], 0);
       mraa_gpio_write(gpio[2], 0);
       mraa_gpio_write(gpio[3], 0);
       mraa_gpio_write(gpio[4], 0);
        printf("off\n");

        sleep(1);
       mraa_gpio_write(gpio[0], 1);
       mraa_gpio_write(gpio[1], 1);
       mraa_gpio_write(gpio[2], 1);
       mraa_gpio_write(gpio[3], 1);
       mraa_gpio_write(gpio[4], 1);
        printf("on\n");

        sleep(1);
    }

    return r;
}
