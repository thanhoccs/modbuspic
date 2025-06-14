#include <string.h>
#include "delay.h"
#include "modbus-rtu.h"
#include "peripheral/uart/plib_uart1.h"
#include "peripheral/uart/plib_uart2.h"


enum { _STEP_FUNCTION = 0x01, _STEP_META, _STEP_DATA };

/* Private variables */
static uint8_t          slaveid = -1;
const serial_t*         serial;

/* MODBUS MAPPING REGISTERS */
int             nb_bits;
int             start_bits;
int             nb_input_bits;
int             start_input_bits;
int             nb_input_registers;
int             start_input_registers;
int             nb_registers;
int             start_registers;
uint8_t         tab_bits[MODBUS_NB_TAB_BIT];
uint8_t         tab_input_bits[MODBUS_NB_TAB_INPUT_BIT];
uint16_t        tab_input_registers[MODBUS_NB_TAB_INPUT_REGISTER];
uint16_t        tab_registers[MODBUS_NB_TAB_REGISTER];
    
static UART_SERIAL_SETUP setup;
static void uart1_begin(uint32_t baud)
{   
    setup.baudRate  = baud;
    setup.parity    = UART_PARITY_NONE;
    setup.dataWidth = UART_DATA_8_BIT;
    setup.stopBits  = UART_STOP_1_BIT;
    
    UART1_SerialSetup(&setup, UART1_FrequencyGet());
}

static size_t uart1_available(void)
{
    return UART1_ReadCountGet();
}

static uint8_t uart1_read(void)
{
    uint8_t c;
   
    UART1_Read(&c, 1);
    return c;
}

static void uart1_write(uint8_t* buf, const size_t size)
{
    UART1_Write(buf, size);
}

const serial_t uart1 = {
    .name       = "UART1",
    .begin      = uart1_begin,
    .available  = uart1_available,
    .read       = uart1_read,
    .write      = uart1_write,
};


static uint16_t crc16(uint8_t *req, uint8_t req_length)
{
    uint8_t j;
    uint16_t crc;

    crc = 0xFFFF;
    while (req_length--) {
        crc = crc ^ *req++;
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }

    return (crc << 8 | crc >> 8);
}

/**
 * Check the CRC request message and calculate CRC
 * @param msg request message
 * @param msg_length request length
 * @return message length, -1 if any error
 */
static int check_integrity(uint8_t *msg, uint8_t msg_length)
{
    uint16_t crc_calculated;
    uint16_t crc_received;

    if (msg_length < 2)
        return -1;

    crc_calculated = crc16(msg, msg_length - 2);
    crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

    /* Check CRC of modbus message */
    if (crc_calculated == crc_received) {
        return msg_length;
    } 
    else {
        return -1;
    }
}

/**
 * Build a response basis include slave, function to rsp message
 * @param slave slave id
 * @param function function code
 * @param rsp response buffer
 * @return length of response buffer build (2)
 */
static int build_response_basis(uint8_t slave, uint8_t function, uint8_t *rsp)
{
    rsp[0] = slave;
    rsp[1] = function;

    return MODBUS_RTU_PRESET_RSP_LENGTH;
}

/**
 * Send message to master includes CRC
 * @param msg buffer no CRC
 * @param msg_length buffer length
 */
static void send_msg(uint8_t *msg, uint8_t msg_length)
{
    uint16_t crc = crc16(msg, msg_length);

    msg[msg_length++] = crc >> 8;
    msg[msg_length++] = crc & 0x00FF;

    serial->write(msg, msg_length);
}

/**
 * Build a response exception message
 * @param slave slave id
 * @param function function code
 * @param exception_code exception code
 * @param rsp buffer
 * @return buffer size
 */
static uint8_t build_response_exception(uint8_t slave, uint8_t function,
                                  uint8_t exception_code, uint8_t *rsp)
{
    uint8_t rsp_length;

    rsp_length = build_response_basis(slave, function + 0x80, rsp);

    /* Positive exception code */
    rsp[rsp_length++] = exception_code;

    return rsp_length;
}

/**
 * Flush all buffer receive
 */
static void flush(void)
{
    uint8_t i = 0;   
    while (serial->available() && i++ < MODBUS_MAX_ADU_LENGTH) {
        serial->read();
    }
}

/*
 *  ---------- Request     Indication ----------
 *  | Client | ---------------------->| Server |
 *  ---------- Confirmation  Response ----------
 */

/* Computes the length to read after the function received */
static uint8_t compute_meta_length_after_function(int function)
{
    int length;

    if (function <= MODBUS_FC_WRITE_SINGLE_REGISTER) {
        length = 4;
    } 
    else if (function == MODBUS_FC_WRITE_MULTIPLE_COILS ||
             function == MODBUS_FC_WRITE_MULTIPLE_REGISTERS) {
        length = 5;
    } 
    else if (function == MODBUS_FC_MASK_WRITE_REGISTER) {
        length = 6;
    } 
    else if (function == MODBUS_FC_WRITE_AND_READ_REGISTERS) {
        length = 9;
    } 
    else {
        /* MODBUS_FC_READ_EXCEPTION_STATUS, MODBUS_FC_REPORT_SLAVE_ID */
        length = 0;
    }

    return length;
}

/* Computes the length to read after the meta information (address, count, etc) */
static int
compute_data_length_after_meta(uint8_t *msg)
{
    int function = msg[MODBUS_RTU_HEADER_LENGTH];
    int length;

    switch (function) {
    case MODBUS_FC_WRITE_MULTIPLE_COILS:
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
        length = msg[MODBUS_RTU_HEADER_LENGTH + 5];
        break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS:
        length = msg[MODBUS_RTU_HEADER_LENGTH + 9];
        break;
    default:
        length = 0;
    }
    length += MODBUS_RTU_CHECKSUM_LENGTH;

    return length;
}

/**
 * MODBUS listen message from master
 * @param req buffer
 * @return buffer size
 */
static int mb_recv(uint8_t *req)
{
    uint8_t i;
    uint8_t length_to_read;
    uint8_t msg_length;
    uint8_t step;

    /* We need to analyse the message step by step.  At the first step, we want
     * to reach the function code because all packets contain this
     * information. */
    step = _STEP_FUNCTION;
    length_to_read = MODBUS_RTU_HEADER_LENGTH + 1;

    msg_length = 0;
    while (length_to_read != 0) {
        if (!serial->available()) {
            i = 0;
            while (!serial->available()) {
                if (++i == MODBUS_RESPONSE_BYTE_TIMEOUT) {
                    /* Too late, bye bye */
                    return -1 - MODBUS_INFORMATIVE_RX_TIMEOUT;
                }
                delay(1);
            }
        }
        
        req[msg_length] = serial->read();
        /* Moves the pointer to receive other data */
        msg_length++;
        /* Computes remaining bytes */
        length_to_read--;

        if (length_to_read == 0) {
            if (req[MODBUS_RTU_HEADER_LENGTH - 1] != slaveid 
                    && req[MODBUS_RTU_HEADER_LENGTH - 1] != MODBUS_BROADCAST_ADDRESS) {
                flush();
                return -1 - MODBUS_INFORMATIVE_NOT_FOR_US;
            }

            switch (step) {
            case _STEP_FUNCTION:
                /* Function code position */
                length_to_read = compute_meta_length_after_function(req[MODBUS_RTU_HEADER_LENGTH]);
                if (length_to_read != 0) {
                    step = _STEP_META;
                    break;
                } /* else switches straight to the next step */
            case _STEP_META:
                length_to_read = compute_data_length_after_meta(req);
                if ((msg_length + length_to_read) > MODBUS_MAX_ADU_LENGTH) {
                    return -1;
                }
                step = _STEP_DATA;
                break;
            default:
                break;
            }
        }
    }
    return check_integrity(req, msg_length);
}

/* Computes the length of the expected response including checksum */
static unsigned int compute_response_length_from_request(uint8_t *req)
{
    int length;
    const int offset = MODBUS_RTU_HEADER_LENGTH;

    switch (req[offset]) {
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS: {
        /* Header + nb values (code from write_bits) */
        int nb = (req[offset + 3] << 8) | req[offset + 4];
        length = 2 + (nb / 8) + ((nb % 8) ? 1 : 0);
    } break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS:
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
        /* Header + 2 * nb values */
        length = 2 + 2 * (req[offset + 3] << 8 | req[offset + 4]);
        break;
    case MODBUS_FC_READ_EXCEPTION_STATUS:
        length = 3;
        break;
    case MODBUS_FC_REPORT_SLAVE_ID:
        /* The response is device specific (the header provides the
           length) */
        return MSG_LENGTH_UNDEFINED;
    case MODBUS_FC_MASK_WRITE_REGISTER:
        length = 7;
        break;
    default:
        length = 5;
    }

    return offset + length + MODBUS_RTU_CHECKSUM_LENGTH;
}

/**
 * Build response io status
 * @param tab_io_status table bits
 * @param address start address
 * @param nb amount
 * @param rsp response buffer
 * @param offset
 * @return offset
 */
static int response_io_status(uint8_t *tab_io_status, 
                              int address, int nb, uint8_t *rsp, int offset)
{
    int shift = 0;
    int one_byte = 0;
    int i;

    for (i = address; i < address + nb; i++) {
        one_byte |= tab_io_status[i] << shift;
        if (shift == 7) {
            /* Byte is full */
            rsp[offset++] = one_byte;
            one_byte = shift = 0;
        } 
        else {
            shift++;
        }
    }

    if (shift != 0)
        rsp[offset++] = one_byte;

    return offset;
}


/**
 * Reply to master
 * @param req request message
 * @param req_length size
 */
static void mb_reply(uint8_t *req, uint8_t req_length)
{
    int offset;
    uint8_t slave;
    uint8_t function;
    uint16_t address;
    uint8_t rsp[MODBUS_MAX_ADU_LENGTH];
    uint8_t rsp_length = 0;
    
    offset              = MODBUS_RTU_HEADER_LENGTH;
    slave               = req[offset - 1];
    function            = req[offset];
    address             = (req[offset + 1] << 8) + req[offset + 2];
      
    if (slave != slaveid && slave != MODBUS_BROADCAST_ADDRESS) {
        return;
    }

    switch (function) {
        case MODBUS_FC_READ_COILS:
        case MODBUS_FC_READ_DISCRETE_INPUTS: {
            unsigned int is_input = (function == MODBUS_FC_READ_DISCRETE_INPUTS);
            int start_bit = is_input ? start_input_bits : start_bits;
            int nb_bit = is_input ? nb_input_bits : nb_bits;
            uint8_t *tab = is_input ? tab_input_bits : tab_bits;
            int nb = (req[offset + 3] << 8) + req[offset + 4];
            int mapping_address = address - start_bit;

            if (nb < 1 || MODBUS_MAX_READ_BITS < nb) {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            } 
            else if (mapping_address < 0 || (mapping_address + nb) > nb_bit) {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            } 
            else {
                rsp_length = build_response_basis(slave, function, rsp);
                rsp[rsp_length++] = (nb / 8) + ((nb % 8) ? 1 : 0);
                rsp_length =
                    response_io_status(tab, mapping_address, nb, rsp, rsp_length);
            }
        } break;
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS: {
            unsigned int is_input = (function == MODBUS_FC_READ_INPUT_REGISTERS);
            int start_reg = is_input ? start_input_registers : start_registers;
            int nb_reg = is_input ? nb_input_registers : nb_registers;
            uint16_t *tab = is_input ? tab_input_registers : tab_registers;
            int nb = (req[offset + 3] << 8) + req[offset + 4];
            int mapping_address = address - start_reg;

            if (nb < 1 || MODBUS_MAX_READ_REGISTERS < nb) {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            } else if (mapping_address < 0 || (mapping_address + nb) > nb_reg) {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            } 
            else {
                int i;
                rsp_length = build_response_basis(slave, function, rsp);
                rsp[rsp_length++] = nb << 1;
                for (i = mapping_address; i < mapping_address + nb; i++) {
                    rsp[rsp_length++] = tab[i] >> 8;
                    rsp[rsp_length++] = tab[i] & 0xFF;
                }
            }
        } break;
        case MODBUS_FC_WRITE_SINGLE_COIL: {
            int mapping_address = address - start_bits;
            if (mapping_address < 0 || mapping_address >= nb_bits) {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
                break;
            }

            /* This check is only done here to ensure using memcpy is safe. */
            rsp_length = compute_response_length_from_request((uint8_t*)req);
            if (rsp_length != req_length) {
                /* Bad use of modbus_reply */
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
                break;
            }

            /* Don't copy the CRC, if any, it will be computed later (even if identical to the
             * request) */
            rsp_length -= MODBUS_RTU_CHECKSUM_LENGTH;

            int data = (req[offset + 3] << 8) + req[offset + 4];
            if (data == 0xFF00 || data == 0x0) {
                /* Apply the change to mapping */
                tab_bits[mapping_address] = data ? 1 : 0;
                /* Prepare response */
                memcpy(rsp, req, rsp_length);
            } 
            else {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            }
        } break;
        case MODBUS_FC_WRITE_SINGLE_REGISTER: {
            int mapping_address = address - start_registers;

            if (mapping_address < 0 || mapping_address >= nb_registers) {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
                break;
            }

            rsp_length = compute_response_length_from_request((uint8_t *) req);
            if (rsp_length != req_length) {
                /* Bad use of modbus_reply */
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
                break;
            }
            int data = (req[offset + 3] << 8) + req[offset + 4];
            tab_registers[mapping_address] = data;

            rsp_length -= MODBUS_RTU_CHECKSUM_LENGTH;
            memcpy(rsp, req, rsp_length);
        } break;
        case MODBUS_FC_WRITE_MULTIPLE_COILS: {
            int nb = (req[offset + 3] << 8) + req[offset + 4];
            int nb_bit = req[offset + 5];
            int mapping_address = address - start_bits;

            if (nb < 1 || MODBUS_MAX_WRITE_BITS < nb || nb_bit * 8 < nb) {
                /* May be the indication has been truncated on reading because of
                 * invalid address (eg. nb is 0 but the request contains values to
                 * write) so it's necessary to flush. */
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            } 
            else if (mapping_address < 0 || (mapping_address + nb) > nb_bits) {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            } 
            else {
                /* 6 = byte count */
                modbus_set_bits_from_bytes(tab_bits, mapping_address, nb, &req[offset + 6]);

                rsp_length = build_response_basis(slave, function, rsp);
                /* 4 to copy the bit address (2) and the quantity of bits */
                memcpy(rsp + rsp_length, req + rsp_length, 4);
                rsp_length += 4;
            }
        } break;
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
            int nb = (req[offset + 3] << 8) + req[offset + 4];
            int nb_bytes = req[offset + 5];
            int mapping_address = address - start_registers;

            if (nb < 1 || MODBUS_MAX_WRITE_REGISTERS < nb || nb_bytes != nb * 2) {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
            } 
            else if (mapping_address < 0 || (mapping_address + nb) > nb_registers) {
                rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
            } 
            else {
                int i, j;
                for (i = mapping_address, j = 6; i < mapping_address + nb; i++, j += 2) {
                    /* 6 and 7 = first value */
                    tab_registers[i] = (req[offset + j] << 8) + req[offset + j + 1];
                }

                rsp_length = build_response_basis(slave, function, rsp);
                /* 4 to copy the address (2) and the no. of registers */
                memcpy(rsp + rsp_length, req + rsp_length, 4);
                rsp_length += 4;
            }
        } break;                  
        default:
            rsp_length = 
                        build_response_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_FUNCTION, rsp);
            break;
    }
    
    send_msg(rsp, rsp_length);
}


void mb_set_slave(uint8_t slave)
{
    if (slave > 0 && slave < 247) {
        slaveid = slave;
    }
}

void mb_init(int baud)
{
    /* Initialize registers mapping */
    nb_bits                 = MODBUS_NB_TAB_BIT; 
    start_bits              = 0;    
    nb_input_bits           = MODBUS_NB_TAB_INPUT_BIT;
    start_input_bits        = 0;
    nb_input_registers      = MODBUS_NB_TAB_INPUT_REGISTER;
    start_input_registers   = 0;
    nb_registers            = MODBUS_NB_TAB_REGISTER;
    start_registers         = 0;
    
    /* Setup serial line */
    serial = &uart1;
    serial->begin(baud);
}


/**
 * MODBUS exchange loop
 * @return 0 if a slave filtering, -1 undefined error, -2 exception illegal function
 */
int mb_loop(void)
{
    int rc = 0;    
    uint8_t req[MODBUS_MAX_ADU_LENGTH];

    if (serial->available()) {
        rc = mb_recv(req);
        if (rc > 0) {
            mb_reply(req, rc);
        }
    }

    /* Returns a positive value if successful,
       0 if a slave filtering has occured,
       -1 if an undefined error has occured,
       -2 for MODBUS_EXCEPTION_ILLEGAL_FUNCTION
       etc */
    return rc;
}
