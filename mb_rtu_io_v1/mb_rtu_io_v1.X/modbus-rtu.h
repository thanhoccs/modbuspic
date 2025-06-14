/* 
 * File:   modbus-rtu.h
 * Author: thanho
 *
 * Created on June 13, 2025, 8:48 PM
 */

#ifndef MODBUS_RTU_H
#define	MODBUS_RTU_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define MODBUS_BROADCAST_ADDRESS                    0

/* Protocol exceptions */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION           0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS       0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE         0x03
#define MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE    0x04
#define MODBUS_EXCEPTION_ACKNOWLEDGE                0x05
#define MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY       0x06
#define MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE       0x07
#define MODBUS_EXCEPTION_MEMORY_PARITY              0x08
#define MODBUS_EXCEPTION_NOT_DEFINED                0x09
#define MODBUS_EXCEPTION_GATEWAY_PATH               0x0A
#define MODBUS_EXCEPTION_GATEWAY_TARGET             0x0B
#define MODBUS_EXCEPTION_MAX                        0x0C

/* Supported function codes */
#define MODBUS_FC_READ_COILS                        0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS              0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS            0x03
#define MODBUS_FC_READ_INPUT_REGISTERS              0x04
#define MODBUS_FC_WRITE_SINGLE_COIL                 0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER             0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS             0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS              0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS          0x10
#define MODBUS_FC_REPORT_SLAVE_ID                   0x11
#define MODBUS_FC_MASK_WRITE_REGISTER               0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS          0x17


#define MODBUS_MAX_READ_BITS                        2000
#define MODBUS_MAX_WRITE_BITS                       1968
#define MODBUS_MAX_READ_REGISTERS                   125
#define MODBUS_MAX_WRITE_REGISTERS                  123
#define MODBUS_MAX_WR_WRITE_REGISTERS               121
#define MODBUS_MAX_WR_READ_REGISTERS                125
#define MODBUS_MAX_PDU_LENGTH                       253
#define MODBUS_MAX_ADU_LENGTH                       260


/* Size of registers mapping */
#define MODBUS_NB_TAB_BIT                           500
#define MODBUS_NB_TAB_INPUT_BIT                     500
#define MODBUS_NB_TAB_INPUT_REGISTER                500
#define MODBUS_NB_TAB_REGISTER                      500

#define MSG_LENGTH_UNDEFINED                        -1
/* MODBUS RTU */
#define MODBUS_RTU_CHECKSUM_LENGTH                  2
#define MODBUS_RTU_HEADER_LENGTH                    1
#define MODBUS_RTU_PRESET_RSP_LENGTH                2
#define MODBUS_INFORMATIVE_NOT_FOR_US               4
#define MODBUS_INFORMATIVE_RX_TIMEOUT               5


/* MODBUS TIMEOUT */
#define MODBUS_RESPONSE_BYTE_TIMEOUT                10

#ifdef	__cplusplus
extern "C" {
#endif

/* Backend serial line */    
typedef struct _serial_t {
    const char* name;
    void        (*begin)(uint32_t baud);
    size_t      (*available)(void);
    uint8_t     (*read)(void);
    void        (*write)(uint8_t* buf, const size_t size);
} serial_t;


/* Global Variables */    
extern int             nb_bits;
extern int             start_bits;
extern int             nb_input_bits;
extern int             start_input_bits;
extern int             nb_input_registers;
extern int             start_input_registers;
extern int             nb_registers;
extern int             start_registers;
extern uint8_t         tab_bits[MODBUS_NB_TAB_BIT];
extern uint8_t         tab_input_bits[MODBUS_NB_TAB_INPUT_BIT];
extern uint16_t        tab_input_registers[MODBUS_NB_TAB_INPUT_REGISTER];
extern uint16_t        tab_registers[MODBUS_NB_TAB_REGISTER];


void mb_set_slave(uint8_t slave);
void mb_init(int baud);
int mb_loop(void);


/**
 * UTILS FUNCTIONS
 **/

#define MODBUS_GET_HIGH_BYTE(data) (((data) >> 8) & 0xFF)
#define MODBUS_GET_LOW_BYTE(data)  ((data) & 0xFF)
#define MODBUS_GET_INT64_FROM_INT16(tab_int16, index)                                  \
    (((int64_t) tab_int16[(index)] << 48) | ((int64_t) tab_int16[(index) + 1] << 32) | \
     ((int64_t) tab_int16[(index) + 2] << 16) | (int64_t) tab_int16[(index) + 3])
#define MODBUS_GET_INT32_FROM_INT16(tab_int16, index) \
    (((int32_t) tab_int16[(index)] << 16) | (int32_t) tab_int16[(index) + 1])
#define MODBUS_GET_INT16_FROM_INT8(tab_int8, index) \
    (((int16_t) tab_int8[(index)] << 8) | (int16_t) tab_int8[(index) + 1])
#define MODBUS_SET_INT16_TO_INT8(tab_int8, index, value)            \
    do {                                                            \
        ((int8_t *) (tab_int8))[(index)] = (int8_t) ((value) >> 8); \
        ((int8_t *) (tab_int8))[(index) + 1] = (int8_t) (value);    \
    } while (0)
#define MODBUS_SET_INT32_TO_INT16(tab_int16, index, value)              \
    do {                                                                \
        ((int16_t *) (tab_int16))[(index)] = (int16_t) ((value) >> 16); \
        ((int16_t *) (tab_int16))[(index) + 1] = (int16_t) (value);     \
    } while (0)
#define MODBUS_SET_INT64_TO_INT16(tab_int16, index, value)                  \
    do {                                                                    \
        ((int16_t *) (tab_int16))[(index)] = (int16_t) ((value) >> 48);     \
        ((int16_t *) (tab_int16))[(index) + 1] = (int16_t) ((value) >> 32); \
        ((int16_t *) (tab_int16))[(index) + 2] = (int16_t) ((value) >> 16); \
        ((int16_t *) (tab_int16))[(index) + 3] = (int16_t) (value);         \
    } while (0)

void modbus_set_bits_from_byte(uint8_t *dest, int idx, const uint8_t value);
void modbus_set_bits_from_bytes(uint8_t *dest,
                                           int idx,
                                           unsigned int nb_bits,
                                           const uint8_t *tab_byte);
uint8_t modbus_get_byte_from_bits(const uint8_t *src,
                                             int idx,
                                             unsigned int nb_bits);
float modbus_get_float(const uint16_t *src);
float modbus_get_float_abcd(const uint16_t *src);
float modbus_get_float_dcba(const uint16_t *src);
float modbus_get_float_badc(const uint16_t *src);
float modbus_get_float_cdab(const uint16_t *src);

void modbus_set_float(float f, uint16_t *dest);
void modbus_set_float_abcd(float f, uint16_t *dest);
void modbus_set_float_dcba(float f, uint16_t *dest);
void modbus_set_float_badc(float f, uint16_t *dest);
void modbus_set_float_cdab(float f, uint16_t *dest);

#ifdef	__cplusplus
}
#endif

#endif	/* MODBUS_RTU_H */

