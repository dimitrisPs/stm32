#include "nrf24.h"


static GPIO_TypeDef *CS_PORT =GPIOA;
static uint16_t CS_PIN =GPIO_PIN_4;
static SPI_HandleTypeDef *SPI;



//alpha




void nrf24_setRole(uint8_t nrf24_role)
{
    //read modofy write
    uint8_t currentVal;
    //first read the current configuration
    nrf24_readCommandRegister(CONFIG, &currentVal,1);
    //check if the register in the radio needs modification
    if (nrf24_role)
    {
        if (currentVal & NRF24_PRIMARY_TX_RX_BIT)
        {//if it is already transmitter
            return;
        }
        else
        {
            //set the
            currentVal|=NRF24_PRIMARY_TX_RX_BIT;
            nrf24_writeCommandRegister(CONFIG, &currentVal,1);
            return;
        }
    }
    else
    {
        if (currentVal & NRF24_PRIMARY_TX_RX_BIT)
        {
            currentVal &= (!NRF24_PRIMARY_TX_RX_BIT);
            nrf24_writeCommandRegister(CONFIG, &currentVal,1);
            return;
        }
        else
        {
            return;
        }
    }
}

uint8_t nrf24_flushTx()
{
    return spiSendCommand(FLUSH_TX);
}

uint8_t nrf24_flushRx()
{
    spiSendCommand(FLUSH_RX);
}

uint8_t nrf24_flushBuffers()
{
    nrf24_flushRx();
    return nrf24_flushTx();
}

uint8_t nrf24_forceRetransmitLast()
{
    return spiSendCommand(REUSE_TX_PL);
}

uint8_t nrf24_getStatus()
{
    //the first byte that nrf24 clocks out is status register
    return spiSendCommand(NOP);
}

void nrf24_setChannel(uint8_t channel)
{
    if (channel >126)
    {
        channel=126;
    }
    if (channel<0)
    {
        channel=0;
    }
    nrf24_writeCommandRegister(RF_CH, &channel,1);
}

void nrf24_setTxAddress(uint8_t txAddr[],uint8_t nrf24_address_width_x, bool withAcknowledge)
{
    if (nrf24_address_width_x<1 || nrf24_address_width_x>3 )
        return;
    nrf24_writeCommandRegister(SETUP_AW, &nrf24_address_width_x, 1);//set address width

    //set address in tx register
    nrf24_writeCommandRegister(TX_ADDR, txAddr, nrf24_address_width_x + 2);//+2 is because all the defines
    //have an offset of 2


    //if auto ack option is selected, also set the same address in rx pipe 0 to get the
    //ack msg from the receiver
    if (withAcknowledge)
    {
        nrf24_writeCommandRegister(RX_ADDR_P0, txAddr, nrf24_address_width_x + 2);
        nrf24_autoACK(NRF24_AUTO_ACK_PIPE_0, NRF24_AUTO_ACK_STATE_ENABLE); //enable pipe 0 auto ack
    }

}








//private functions

uint8_t spiWriteBytes (uint8_t address, uint8_t  *inputArray,uint8_t len)
{
    uint8_t status,readAdrr = (address & 0x1F)|0x20;//read adresses are in the form of
    //001AAAAA. by making the above AND,OR operations we clear the first 2 bits and
    //set the third one of the address

    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(SPI, &readAdrr, &status, 1, TIMEOUT);

    if (len>1)
        HAL_SPI_TransmitReceive(SPI, &readAdrr, &inputArray, len, TIMEOUT);

    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

    return status;
}

uint8_t spiReadBytes(uint8_t address, uint8_t  *ouputArray,uint8_t len)
{
    uint8_t status,readAdrr = address & 0x1F;//read adresses are in the form of
    //000AAAAA. by making the above and operation we clear the first 3 bits

    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

    //by reading-writing one byte we specify the read register and the chip
    //reports back with the status register.
    HAL_SPI_TransmitReceive(SPI, &readAdrr, &status, 1, TIMEOUT);
    //now that the chips has the adress, every trnasaction will return us the
    //disired data. (page 49 of datasheet nrf24L01+)
    if (len>1)
        HAL_SPI_TransmitReceive(SPI, &readAdrr, &ouputArray, len, TIMEOUT);

    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

    return status;
}

uint8_t spiSendCommand(uint8_t command)
{
    uint8_t status;
    HAL_SPI_TransmitReceive(SPI, &command, &status, 1, TIMEOUT);
    return status;
}


void nrf24_setDataRate(uint8_t nrf24_data_rate_x)
{
    //read moify write
    uint8_t rfRegVal;
    nrf24_readCommandRegisters(RF_SETUP,&rfRegVal,1);

    if(nrf24_data_rate_x == NRF24_DATA_RATE_250Kbps)
    {
        //we need to set bit 5 and clear bit 3
        rfRegVal |= NRF24_DATA_RATE_LOW_BIT; //set bit 5
        rfRegVal &= !NRF24_DATA_RATE_HIGH_BIT;//clear bit 3
    }
    else if(nrf24_data_rate_x == NRF24_DATA_RATE_1Mbps)
    {
        //we need to clear bit 5 and bit 3
        rfRegVal &= !NRF24_DATA_RATE_LOW_BIT; //set bit 5
        rfRegVal &= !NRF24_DATA_RATE_HIGH_BIT;//clear bit 3
    }
    else if(nrf24_data_rate_x == NRF24_DATA_RATE_2Mbps)
    {
        //we need to clear bit 5 and set bit 3
        rfRegVal &= !NRF24_DATA_RATE_LOW_BIT; //set bit 5
        rfRegVal |= NRF24_DATA_RATE_HIGH_BIT;//clear bit 3
    }
    nrf24_writeCommandRegister(RF_SETUP,&rfRegVal,1);
}

void nrf24_setRfPower(uint8_t nrf24_rf_power_output_x)
{
    uint8_t rfRegVal;
    nrf24_readCommandRegister(RF_SETUP, &rfRegVal,1);
    if (nrf24_rf_power_output_x == NRF24_POWER_OUTPUT_0dBm)
    {
        rfRegVal |= 0x06; //set bit 1 and 2
    }
    else if(nrf24_rf_power_output_x == NRF24_POWER_OUTPUT_MINUS_6dBm)
    {
        rfRegVal &= !0X2;//clear bit 1
        rfRegVal |= 0X4;//set bit 2
    }
    else if(nrf24_rf_power_output_x == NRF24_POWER_OUTPUT_MINUS_12dBm)
    {
        rfRegVal |= 0X2; //set bit 1
        rfRegVal &= !0X4;//clear bit 2
    }
    else if(nrf24_rf_power_output_x == NRF24_POWER_OUTPUT_MINUS_18dBm)
    {
        rfRegVal &= !0x06; //clear bit 1 and 2
    }
}



void nrf24_autoACK(uint8_t nrf24_auto_ack_pipe_x, bool nrf24_auto_ack_state_x)
{
    //read modify write;
    uint8_t currentVal;
    nrf24_readCommandRegister(EN_AA, &currentVal, 1);
    if (nrf24_auto_ack_state_x==NRF24_AUTO_ACK_STATE_ENABLE)//in case we want to enable auto ack in pipes x
    {
        currentVal |= nrf24_auto_ack_pipe_x;
    }
    else
    {
        currentVal &= !nrf24_auto_ack_pipe_x;
    }
    nrf24_writeCommandRegister(EN_AA, &currentVal, 1);
}

void nrf24_rxPipeEnable(uint8_t nrf24_rx_pipe_x, bool nrf24_rx_pipe_state_x)
{
    //read modify write;
    uint8_t currentVal;
    nrf24_readCommandRegister(EN_RXADDR, &currentVal, 1);
    if (nrf24_rx_pipe_x==NRF24_RX_PIPE_STATE_ENABLE)//in case we want to enable auto ack in pipes x
    {
        currentVal |= nrf24_rx_pipe_x;
    }
    else
    {
        currentVal &= !nrf24_rx_pipe_x;
    }
    nrf24_writeCommandRegister(EN_RXADDR, &currentVal, 1);
}

void nrf24_setAdressLen(uint8_t nrf24_adress_width_x)
{
    if (nrf24_adress_width_x<3 || nrf24_adress_width_x>5)
        return;
    nrf24_writeCommandRegister(SETUP_AW, &nrf24_adress_width_x,1);
}

void nrf24_powerUp()
{
    uint8_t current;
    nrf24_readCommandRegister(CONFIG, &current, 1);
    current |= NRF24_POWER_UP_BIT;
    nrf24_writeCommandRegister(CONFIG, &current, 1);
}

void nrf24_powerDown()
{
    uint8_t current;
    nrf24_readCommandRegister(CONFIG, &current, 1);
    current &= !NRF24_POWER_UP_BIT;
    nrf24_writeCommandRegister(CONFIG, &current, 1);

}

void nrf24_crc(uint8_t nrf24_crc_state_x,uint8_t nrf24_crc_encoding_bits_x)
{
    uint8_t current;
    nrf24_readCommandRegister(CONFIG, &current, 1);
    if (nrf24_crc_state_x==NRF24_CRC_STATE_ENABLE)
    {
        current|=NRF24_CRC_ENABLE_BIT;
        if (nrf24_crc_encoding_bits_x==NRF24_CRC_ENCODING_BITS_1)
        {
            current &= !NRF24_CRC_ENCODING_BIT;
        }
        else
        {
            current |= NRF24_CRC_ENCODING_BIT;
        }
    }
    else
    {
        current &= !NRF24_CRC_ENABLE_BIT;
    }
    nrf24_writeCommandRegister(CONFIG, &current, 1);
}

void nrf24_irqMaskSetup(uint8_t nrf24_irq_mask_x,uint8_t nrf24_irq_mask_state_x)
{
    uint8_t current;
    nrf24_readCommandRegister(CONFIG, &current, 1);
    if (nrf24_irq_mask_state_x == NRF24_IRQ_MASK_STATE_ENABLE)
    {
        current |=nrf24_irq_mask_x;
    }
    else
    {
        current &= !nrf24_irq_mask_x;
    }
    nrf24_writeCommandRegister(CONFIG, &current, 1);
}

void nrf24_clearIntr(uint8_t nrf24_interrupt_x)
{
    uint8_t current=nrf24_getStatus();
    current &= !nrf24_interrupt_x;
    nrf24_writeCommandRegister(STATUS, &current, 1);
}

bool nrf24_isTxFull()
{
    uint8_t status=nrf24_getStatus();
    if (NRF24_STATUS_TX_FULL & status)
        return true;
    return false;
}

uint8_t nrf24_getlostPacketCount()
{
    uint8_t current;
    nrf24_readCommandRegister(OBSERVE_TX, &current, 1);
    current= current >> 4;//shift bits 7:4 4 possitions to replace bits 3:0
    //current &= 0x0f;      //clear bits 7:4 in case shifting fills them with 0
    return current;
}

uint8_t nrf24_getRetransmitNo()
{
    uint8_t current;
    nrf24_readCommandRegister(OBSERVE_TX, &current, 1);
    current &= 0x0f;      //keep only bits 3:0
    return current;
}

bool nrf24_isReceivedPowerLow()
{
    uint8_t current;
    nrf24_readCommandRegister(RPD, &current, 1);
    return current;
}

//pre-apha



void nrf24_writeCommandRegister(uint8_t nrf24_reg, uint8_t *value, uint8_t len)
{
    spiWriteBytes(nrf24_reg, value, len);
}

uint8_t nrf24_readCommandRegister(uint8_t nrf24_reg, uint8_t *output, uint8_t len)
{
    spiReadBytes(nrf24_reg, output, len);
    return output[0];
}

void nrf24_write(uint8_t *payload, uint8_t len)
{
    uint8_t cmd_addr=W_TX_PAYLOAD;
    HAL_SPI_TransmitReceive(SPI,&cmd_addr , NULL, 1, TIMEOUT);
    HAL_SPI_TransmitReceive(SPI, payload , NULL, len, TIMEOUT);
}

uint8_t nrf24_read(uint8_t *data)
{
    uint8_t cmd_addr=R_RX_PAYLOAD;
    HAL_SPI_TransmitReceive(SPI, &cmd_addr , NULL, 1, TIMEOUT);
    HAL_SPI_TransmitReceive(SPI, data , data, 32, TIMEOUT);
}


//have to implement ack payload and rx payload width.


// low level io operation with the use of those function we can port easyly the
// library to other platforms.
void nrf24_init(SPI_HandleTypeDef *spiInstance, GPIO_TypeDef *csPort, uint16_t csPin)
{
    CS_PORT=csPort;
    CS_PIN=csPin;
    SPI=spiInstance;
}

#define NRF24_ROLE_TRANSMITTER 1
#define NRF24_ROLE_RECEIVER 0










void nrf24_setRetransmissionDelay(uint8_t nrf24_retransmit_delay)
{
    //read modify write
    uint8_t currentValue;
    currentValue=nrf24_readCommandRegister(SETUP_RETR);

    if (nrf24_retransmit_delay >NRF24_RETRANSMIT_DELAY_4000us)
    {
        nrf24_retransmit_delay=NRF24_RETRANSMIT_DELAY_4000us;
    }

    currentValue&=0x0F;
    currentValue|=nrf24_retransmit_delay;
    nrf24_writeCommandRegister(SETUP_RETR, currentValue);
}

void nrf24_setRetransmissionCount(uint8_t nrf24_retransmit_count)
{
    //read modify write
    uint8_t currentValue;
    currentValue=nrf24_readCommandRegister(SETUP_RETR);

    if (nrf24_retransmit_count >NRF24_RETRANSMIT_TIMES_15)
    {
        nrf24_retransmit_count=NRF24_RETRANSMIT_TIMES_15;
    }

    currentValue&=0xF0;
    currentValue|=nrf24_retransmit_count;
    nrf24_writeCommandRegister(SETUP_RETR, currentValue);
}

void nrf24_setAdressLen(uint8_t nrf24_adress_width_x)
{
    if (nrf24_adress_width_x<3 || nrf24_adress_width_x>5)
        return;
    nrf24_writeCommandRegister(SETUP_AW, &nrf24_adress_width_x,1);
}
