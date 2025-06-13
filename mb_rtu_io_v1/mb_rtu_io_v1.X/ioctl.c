#include "ioctl.h"
#include "modbus-rtu.h"
#include "peripheral/gpio/plib_gpio.h"

static void ioctl_mapping_tab_bits(void)
{
    uint8_t bit = 0;
    
    for (bit = start_bits; bit < nb_bits; bit++) {
        switch (bit) {
            case 0:
                if (tab_bits[bit]) GPIO_LED_1_Set(); else GPIO_LED_1_Clear();
                break;
            case 1:
                if (tab_bits[bit]) GPIO_LED_2_Set(); else GPIO_LED_2_Clear();
                break;
            case 2:
                if (tab_bits[bit]) GPIO_LED_3_Set(); else GPIO_LED_3_Clear();
                break;
            default:
                break;
        }
    }
}


void ioctl_init(void)
{
    ;
}


void ioctl_loop(void)
{
    ioctl_mapping_tab_bits();
}



