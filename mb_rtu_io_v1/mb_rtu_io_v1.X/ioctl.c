#include "ioctl.h"
#include "modbus-rtu.h"
#include "peripheral/gpio/plib_gpio.h"


#define _IOCTL_NB_BITS      3



static void ioctl_mapping_tab_bits(void)
{
    uint8_t nb_bit = 0;
    
    for (nb_bit = start_bits; nb_bit < nb_bits; nb_bit++) {
        if (nb_bit >= _IOCTL_NB_BITS) break;
        switch (nb_bit) {
            case 0:
                if (tab_bits[nb_bit]) GPIO_LED_1_Set(); else GPIO_LED_1_Clear();
                break;
            case 1:
                if (tab_bits[nb_bit]) GPIO_LED_2_Set(); else GPIO_LED_2_Clear();
                break;
            case 2:
                if (tab_bits[nb_bit]) GPIO_LED_3_Set(); else GPIO_LED_3_Clear();
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



