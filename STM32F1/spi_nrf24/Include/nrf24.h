#ifndef __NRF24_H
#define __NRF24_H

#include "nRF24L01.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"

typedef enum bl{true=1,false=0}bool;



#define TIMEOUT 50
//config register 0x00

#define NRF24_IRQ_MASK_RX_FULL              0x40 //unmask(default)=0, mask=1
#define NRF24_IRQ_MASK_TX_EMPTY             0x20 //unmask(default)=0, mask=1
#define NRF24_IRQ_MASK_MAX_RETRANSMIT       0x10 //unmask(default)=0, mask=1
#define NRF24_CRC_ENABLE_BIT                0x08 //enabled by default=1, disable=0
#define NRF24_CRC_ENCODING_BIT              0x04 //1BYTE=0, 2BYTES=1
#define NRF24_POWER_UP_BIT                  0x02 //POWER UP=1, POWER DOWN=0
#define NRF24_PRIMARY_TX_RX_BIT             0x02 //PTX=0, PRX=1

#define NRF24_CRC_STATE_ENABLE              0X01
#define NRF24_CRC_STATE_DESABLE             0X00

#define NRF24_CRC_ENCODING_BITS_1           0X00
#define NRF24_CRC_ENCODING_BITS_2           0X01

#define NRF24_IRQ_MASK_STATE_ENABLE         0x00
#define NRF24_IRQ_MASK_STATE_DISABLE        0x01

#define NRF24_ROLE_TRANSMITTER 1
#define NRF24_ROLE_RECEIVER 0


//enable auto acknowledgement register 0x01

#define NRF24_AUTO_ACK_PIPE_0               0X01  //enable(default)=1, disable=0
#define NRF24_AUTO_ACK_PIPE_1               0X02  //enable(default)=1, disable=0
#define NRF24_AUTO_ACK_PIPE_2               0X04  //enable(default)=1, disable=0
#define NRF24_AUTO_ACK_PIPE_3               0X08  //enable(default)=1, disable=0
#define NRF24_AUTO_ACK_PIPE_4               0X10 //enable(default)=1, disable=0
#define NRF24_AUTO_ACK_PIPE_5               0X20 //enable(default)=1, disable=0

#define NRF24_AUTO_ACK_STATE_ENABLE         0X01
#define NRF24_AUTO_ACK_STATE_DESABLE        0X00

//enable rx pipe register 0x02

#define NRF24_RX_PIPE_0                     0X01  //enable=1, disable(default)=0
#define NRF24_RX_PIPE_1                     0X02  //enable=1, disable(default)=0
#define NRF24_RX_PIPE_2                     0X04  //enable=1, disable(default)=0
#define NRF24_RX_PIPE_3                     0X08  //enable=1, disable(default)=0
#define NRF24_RX_PIPE_4                     0X10 //enable(default)=1, disable=0
#define NRF24_RX_PIPE_5                     0X20 //enable(default)=1, disable=0

#define NRF24_RX_PIPE_STATE_ENABLE          0X01
#define NRF24_RX_PIPE_STATE_DESABLE         0X00

//address width register 0x03

#define NRF24_ADDRESS_WIDTH_5               0X03 //default
#define NRF24_ADDRESS_WIDTH_4               0X02 //adress LSBs used as address if
#define NRF24_ADDRESS_WIDTH_3               0X01 //4 or 3 adreess width selected


//automatic retransmission config register 0x04

//one can configure this register with an or operation like
//(NRF24_RETRANSMIT_DELAY_250us | NRF24_RETRANSMIT_TIMES_1) which configures the
//radio to retransmit on fail one time after waitng for an acknowledgment for 250us

#define NRF24_RETRANSMIT_DELAY_250us        0X00 // default
#define NRF24_RETRANSMIT_DELAY_500us        0X10
#define NRF24_RETRANSMIT_DELAY_750us        0X20
#define NRF24_RETRANSMIT_DELAY_1000us       0X30
#define NRF24_RETRANSMIT_DELAY_1250us       0X40
#define NRF24_RETRANSMIT_DELAY_1500us       0X50
#define NRF24_RETRANSMIT_DELAY_1750us       0X60
#define NRF24_RETRANSMIT_DELAY_2000us       0X70
#define NRF24_RETRANSMIT_DELAY_2250us       0X80
#define NRF24_RETRANSMIT_DELAY_2500us       0X90
#define NRF24_RETRANSMIT_DELAY_2750us       0XA0
#define NRF24_RETRANSMIT_DELAY_3000us       0XB0
#define NRF24_RETRANSMIT_DELAY_3250us       0XC0
#define NRF24_RETRANSMIT_DELAY_3500us       0XD0
#define NRF24_RETRANSMIT_DELAY_3750us       0XE0
#define NRF24_RETRANSMIT_DELAY_4000us       0XF0

#define NRF24_RETRANSMIT_DISABLE            0X00
#define NRF24_RETRANSMIT_TIMES_1            0X01
#define NRF24_RETRANSMIT_TIMES_2            0X02
#define NRF24_RETRANSMIT_TIMES_3            0X03  //default
#define NRF24_RETRANSMIT_TIMES_4            0X04
#define NRF24_RETRANSMIT_TIMES_5            0X05
#define NRF24_RETRANSMIT_TIMES_6            0X06
#define NRF24_RETRANSMIT_TIMES_7            0X07
#define NRF24_RETRANSMIT_TIMES_8            0X08
#define NRF24_RETRANSMIT_TIMES_9            0X09
#define NRF24_RETRANSMIT_TIMES_10           0X0A
#define NRF24_RETRANSMIT_TIMES_11           0X0B
#define NRF24_RETRANSMIT_TIMES_12           0X0C
#define NRF24_RETRANSMIT_TIMES_13           0X0D
#define NRF24_RETRANSMIT_TIMES_14           0X0E
#define NRF24_RETRANSMIT_TIMES_15           0X0F

//channel selection register 0x05
//default is channel 2. 127 channels available. by selecting a channel you
//set the frequency of communication between radios.

//rf configure register 0x06

#define NRF24_CONTINUOUS_WAVE_ENABLE        0x80 //enable=1, disable(default)=0
#define NRF24_PPL_FORCE_LOCK                0x10 //lock=1, unlock(default)=0
#define NRF24_DATA_RATE_HIGH_BIT            0x08
#define NRF24_DATA_RATE_LOW_BIT             0x20

//for use as parameter with nrf24_setDataRate

#define NRF24_DATA_RATE_250Kbps             0
#define NRF24_DATA_RATE_1Mbps               1
#define NRF24_DATA_RATE_2Mbps               2

#define NRF24_POWER_OUTPUT_0dBm             0
#define NRF24_POWER_OUTPUT_MINUS_6dBm       1
#define NRF24_POWER_OUTPUT_MINUS_12dBm      2
#define NRF24_POWER_OUTPUT_MINUS_18dBm      3

//status register 0x07

#define NRF24_STATUS_TX_FULL                0X01
#define NRF24_INTR_RX_DATA_READY            0X40
#define NRF24_INTR_TX_DATA_SENT             0X20
#define NRF24_INTR_MAX_RETRANSMITIONS       0X10


uint8_t spiWriteBytes(uint8_t address, uint8_t  *inputArray,uint8_t len);//done

uint8_t spiReadBytes(uint8_t address, uint8_t  *ouputArray,uint8_t len);//done

uint8_t spiSendCommand(uint8_t command);//done


void nrf24_writeCommandRegister(uint8_t nrf24_reg, uint8_t *value, uint8_t len); //done

uint8_t nrf24_readCommandRegister(uint8_t nrf24_reg, uint8_t *output, uint8_t len); //done



uint8_t nrf24_forceRetransmitLast();  //done

//have to implement ack payload and rx payload width.


// low level io operation with the use of those function we can port easyly the
// library to other platforms.
void nrf24_init(SPI_HandleTypeDef *spiInstance, GPIO_TypeDef *csPort, uint16_t csPin);


void nrf24_setRole(uint8_t nrf24_role);//done

uint8_t nrf24_getStatus();//done


uint8_t nrf24_flushTx();//done

uint8_t nrf24_flushRx();//done


uint8_t nrf24_flushBuffers();//done


void nrf24_setChannel(uint8_t channel);//done



void nrf24_setRetransmissionDelay(uint8_t nrf24_retransmit_delay);//done

void nrf24_setRetransmissionCount(uint8_t nrf24_retransmit_count);//done

void nrf24_setAdressLen(uint8_t nrf24_adress_width_x);//done





void nrf24_setDataRate(uint8_t nrf24_data_rate_x);//done //THELEI LIGO MAZEMA ME TA DEFINES

void nrf24_setRfPower(uint8_t nrf24_rf_power_output_x);//done   //THEL;EI LIGO MAZEMA ME TA DEFINES EPISIS

void nrf24_autoACK(uint8_t nrf24_auto_ack_pipe_x, bool nrf24_auto_ack_state_x);//done

void nrf24_rxPipeEnable(uint8_t nrf24_rx_pipe_x, bool nrf24_rx_pipe_state_x);//done

void nrf24_setAdressLen(uint8_t nrf24_adress_width_x);//done

void nrf24_powerUp();//done

void nrf24_powerDown();//done

void nrf24_crc(bool nrf24_crc_state_x,uint8_t nrf24_crc_encoding_bits_x);//done

void nrf24_irqMaskSetup(uint8_t nrf24_irq_mask_x,bool nrf24_irq_mask_state_x);//done

void nrf24_clearIntr(uint8_t nrf24_interrupt_x);//done

//void fifo status

bool nrf24_isTxFull();//done

uint8_t nrf24_getlostPacketCount(); //done

uint8_t nrf24_getRetransmitNo(); //done

bool nrf24_isReceivedPowerLow();//done

void nrf24_setTxAddress(uint8_t txAddr[],uint8_t addrSize, bool withAcknowledge);//done









void nrf24_writePayload(uint8_t *payload, uint8_t len);

uint8_t nrf24_readPayload(uint8_t *data, uint8_t len);

void nrf24_retransmission(bool enable);

void nrf24_setOperatingMode();

void nrf24_setAckPayload();


void nrf24_setRxAddresses(uint8_t rxAddr[5][]);

void nrf24_setRxAddress(uint8_t rxAddr[],uint8_t pipe);


#endif //NRF24_H
