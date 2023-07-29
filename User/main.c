/*
 * PSG Emulator for CH32V003
 * Input I2C (SDA PC1/SCL PC2)
 *  Output: TIM1CH4 (PC4)
 */

#include "debug.h"
//#define SAMPLING_FREQ 48000
#define SAMPLING_FREQ 44100
//#define SAMPLING_FREQ 32000
//#define SAMPLING_FREQ 22000
#define TIME_UNIT 100000000                           // Oscillator calculation resolution = 10nsec
#define SAMPLING_INTERVAL (TIME_UNIT/SAMPLING_FREQ)   // 20.833 usec in 48KHz Sampling freq
#define PSG_CLOCK1 (4000000/2)
#define PSG_CLOCK2 (3579545/2)
#define DMA_TIMEOUT 1000
#define RX_BUFFER_LEN 128

#define PSG_ADDRESS 0x20    // i2c address = 0x10

#define PSG_NUMBERS 2

//#define PSG_DEBUG

/* Global Variable */

uint8_t psg_register[16 * PSG_NUMBERS];
uint32_t psg_osc_interval[3 * PSG_NUMBERS];
uint32_t psg_osc_counter[3 * PSG_NUMBERS];

uint32_t psg_noise_interval[PSG_NUMBERS];
uint32_t psg_noise_counter[PSG_NUMBERS];
uint8_t psg_noise_output[PSG_NUMBERS];
uint32_t psg_noise_seed[PSG_NUMBERS];
uint32_t psg_envelope_interval[PSG_NUMBERS];
uint32_t psg_envelope_counter[PSG_NUMBERS];
uint32_t psg_master_clock = PSG_CLOCK2;
uint16_t psg_master_volume = 0;

uint8_t psg_tone_on[3 * PSG_NUMBERS], psg_noise_on[3 * PSG_NUMBERS];

const uint16_t psg_volume[] = { 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08, 0x09, 0x0b, 0x0d, 0x10, 0x13, 0x17, 0x1b, 0x20,
        0x26, 0x2d, 0x36, 0x40, 0x4c, 0x5a, 0x6b, 0x80, 0x98, 0xb4, 0xd6, 0xff };

uint8_t rxdata[2];
uint8_t rx_buffer[RX_BUFFER_LEN];
volatile uint8_t rxstatus;
volatile uint8_t rx_write_ptr=0;
volatile uint8_t rx_read_ptr=0;

//

void i2cinit(u32 bound, u16 address) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    I2C_InitTypeDef I2C_InitTSturcture = { 0 };

#ifdef CH32V20x_D6
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
#else
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure);
#endif

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture);

    I2C_Cmd( I2C1, ENABLE);

 //   I2C_DMACmd( I2C1, ENABLE);

    I2C_ITConfig(I2C1,I2C_IT_EVT|I2C_IT_ERR,ENABLE);

}

void DMA_Rx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr,
        u16 bufsize) {
    DMA_InitTypeDef DMA_InitStructure = { 0 };

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}

void TIM1_PWMOut_Init(u16 arr, u16 psc, u16 ccp) {
    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init( TIM1, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig( TIM1, ENABLE);
    TIM_Cmd( TIM1, ENABLE);
}

void toneinit(void) {

    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

#ifdef CH32V20x_D6
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure);
#else
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure);
#endif

#ifdef PSG_DEBUG
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    // for debug
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure);
#endif

}

void psg_reset(int flag, int psg_no) {

    if (flag == 0) {
        for (int i = 0; i < 16; i++) {
            psg_register[i + psg_no * 16] = 0;
        }
    } else {
        for (int i = 0; i < 15; i++) {
            psg_register[i + psg_no * 16] = 0;
        }
    }
    psg_register[7 + psg_no * 16] = 0xff;

    psg_noise_interval[psg_no] = UINT32_MAX;
    psg_envelope_interval[psg_no] = UINT32_MAX / 2 - 1;

    if (psg_register[15] == 0) {    // Only PSG0 has Master clock selection flag
        psg_master_clock = PSG_CLOCK2;
    } else {
        psg_master_clock = PSG_CLOCK1;
    }

    for (int i = 0; i < 3; i++) {
        psg_osc_interval[i + psg_no * 3] = UINT32_MAX;
        psg_tone_on[i + psg_no * 3] = 0;
        psg_noise_on[i + psg_no * 3] = 0;
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {

    uint8_t i, psg_no;
    uint32_t freq;

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    // Intialize systick interrupt by sampling frequency

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = SysTicK_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // NVIC_SetPriority(SysTicK_IRQn, 0);
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->SR &= ~(1 << 0);
    SysTick->CMP = (SystemCoreClock / SAMPLING_FREQ) - 1;
    SysTick->CNT = 0;
    SysTick->CTLR = 0xF;




#ifdef PSG_DEBUG
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
#endif

    i2cinit(80000, PSG_ADDRESS);
  //  DMA_Rx_Init( DMA1_Channel7, ( u32 )&I2C1->DATAR, ( u32)&rxbuff, RX_BUFFER_LEN );
  //  DMA_Cmd( DMA1_Channel7, ENABLE );

    toneinit();
    TIM1_PWMOut_Init(256, 0, 255);  // 48MHz * 256 = 5us

    // Initialize PSG
    for (i = 0; i < PSG_NUMBERS; i++) {
        psg_reset(0, i);
        psg_noise_seed[i] = 12345;
    }

    while(1) {

        while(rx_read_ptr==rx_write_ptr) {
 //           printf("%d/%d ",rx_read_ptr,rx_write_ptr);
        }

        rxdata[0]=rx_buffer[rx_read_ptr];
        rxdata[1]=rx_buffer[rx_read_ptr+1];

        rx_read_ptr+=2;
        if(rx_read_ptr>=RX_BUFFER_LEN) rx_read_ptr=0;

#ifdef  PSG_DEBUG
        if(ccount!=0) printf("DMA wait:%d\n\r",ccount);
        printf("%02x %02x\n\r",rxdata[0],rxdata[1]);
#endif

//        I2C1->CTLR1 &= I2C1->CTLR1;

        // store registers

        if(rxdata[0]<0x10*PSG_NUMBERS) {
            psg_register[rxdata[0]]=rxdata[1];
        }

        psg_no=(rxdata[0]&0xf0)>>4;
        if(psg_no<PSG_NUMBERS) {

            switch(rxdata[0]&0xf) {
                case 0:
                case 1:
                freq = psg_master_clock / ( psg_register[0+psg_no*16] + ((psg_register[1+psg_no*16]&0x0f)<<8) );
                freq >>= 4;
                if(freq!=0) {
                    psg_osc_interval[0+psg_no*3] = TIME_UNIT / freq;
                    psg_osc_counter[0+psg_no*3]=0;
                } else {
                    psg_osc_interval[0+psg_no*3]=UINT32_MAX;
                }
                break;
                case 2:
                case 3:
                freq = psg_master_clock / ( psg_register[2+psg_no*16] + ((psg_register[3+psg_no*16]&0x0f)<<8) );
                freq >>= 4;
                if(freq!=0) {
                    psg_osc_interval[1+psg_no*3] = TIME_UNIT / freq;
                    psg_osc_counter[1+psg_no*3]=0;
                } else {
                    psg_osc_interval[1+psg_no*3]=UINT32_MAX;
                }
                break;
                case 4:
                case 5:
                freq = psg_master_clock / ( psg_register[4+psg_no*16] + ((psg_register[5+psg_no*16]&0x0f)<<8) );
                freq >>= 4;
                if(freq!=0) {
                    psg_osc_interval[2+psg_no*3] = TIME_UNIT / freq;
                    psg_osc_counter[2+psg_no*3]=0;
                } else {
                    psg_osc_interval[2+psg_no*3]=UINT32_MAX;
                }
                break;
                case 6:
                freq = psg_master_clock / ( psg_register[6+psg_no*16] & 0x1f );
                freq >>= 4;
                if(freq!=0) {
                    psg_noise_interval[psg_no] = TIME_UNIT / freq;
                    psg_noise_counter[psg_no] = 0;
                } else {
                    psg_noise_interval[psg_no]=UINT32_MAX;
                }
                break;
                case 7:
                psg_tone_on[0+psg_no*3]=((psg_register[7+psg_no*16]&1)==0?1:0);
                psg_tone_on[1+psg_no*3]=((psg_register[7+psg_no*16]&2)==0?1:0);
                psg_tone_on[2+psg_no*3]=((psg_register[7+psg_no*16]&4)==0?1:0);
                psg_noise_on[0+psg_no*3]=((psg_register[7+psg_no*16]&8)==0?1:0);
                psg_noise_on[1+psg_no*3]=((psg_register[7+psg_no*16]&16)==0?1:0);
                psg_noise_on[2+psg_no*3]=((psg_register[7+psg_no*16]&32)==0?1:0);
                break;
                case 0xb:
                case 0xc:
                freq = psg_master_clock / ( psg_register[0xb+psg_no*16] + (psg_register[0xc+psg_no*16]<<8) );
                if(freq!=0) {
                    psg_envelope_interval[psg_no]= TIME_UNIT / freq;
                    psg_envelope_interval[psg_no]<<=5;
                } else {
                    psg_envelope_interval[psg_no]=UINT32_MAX/2-1;
                }
                break;
                case 0xd:
                psg_envelope_counter[psg_no]=0;
                break;
                case 0xf:
                psg_reset(1,psg_no);
            }
        }
    }

}

// I2C interrupt
void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void I2C1_EV_IRQHandler(void) {

    uint16_t dummy;

    if(I2C_GetITStatus(I2C1,I2C_IT_ADDR)==SET) {

//        DMA_Rx_Init( DMA1_Channel7, ( u32 )&I2C1->DATAR, ( u32)&rxdata, 2 );
//        DMA_Cmd( DMA1_Channel7, ENABLE );

        rxstatus=0;

        dummy=I2C1->STAR2;

    } else if(I2C_GetITStatus(I2C1,I2C_IT_RXNE)==SET) {

        if(rxstatus<2) {
            rx_buffer[rx_write_ptr+rxstatus]=I2C_ReceiveData(I2C1);
            rxstatus++;
            if(rxstatus==2) {
                rx_write_ptr+=2;
                if(rx_write_ptr>=RX_BUFFER_LEN) rx_write_ptr=0;
                rxstatus=0;
            }
        }

    } else if (I2C_GetITStatus(I2C1, I2C_IT_STOPF)==SET){
        I2C1->CTLR1 &= I2C1->CTLR1;
    }
}

void I2C1_ER_IRQHandler(void) {

    I2C_DeInit(I2C1);
    i2cinit(80000, PSG_ADDRESS);
}


void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void SysTick_Handler(void) __attribute__((interrupt));

void SysTick_Handler(void) {

    uint32_t pon_count;
    uint16_t master_volume;
    uint8_t tone_output[3 * PSG_NUMBERS], noise_output[3 * PSG_NUMBERS],
            envelope_volume[PSG_NUMBERS];

//    TIM1->CH4CVR = psg_master_volume / (3 * PSG_NUMBERS);

    TIM1->CH4CVR = psg_master_volume;

#ifdef PSG_DEBUG
    GPIO_WriteBit(GPIOC, GPIO_Pin_3, Bit_SET);
#endif

    master_volume = 0;

    // Run Noise generator

    for (int i = 0; i < PSG_NUMBERS; i++) {

        psg_noise_counter[i] += SAMPLING_INTERVAL;
        if (psg_noise_counter[i] > psg_noise_interval[i]) {
            psg_noise_seed[i] = (psg_noise_seed[i] >> 1)
                    | (((psg_noise_seed[i] << 14) ^ (psg_noise_seed[i] << 16))
                            & 0x10000);
            psg_noise_output[i] = psg_noise_seed[i] & 1;
            psg_noise_counter[i] -= psg_noise_interval[i];
        }
        if (psg_noise_output[i] != 0) {
            noise_output[0 + i * 3] = psg_noise_on[0 + i * 3];
            noise_output[1 + i * 3] = psg_noise_on[1 + i * 3];
            noise_output[2 + i * 3] = psg_noise_on[2 + i * 3];
        } else {
            noise_output[0 + i * 3] = 0;
            noise_output[1 + i * 3] = 0;
            noise_output[2 + i * 3] = 0;
        }
    }

    // Run Envelope

    for (int i = 0; i < PSG_NUMBERS; i++) {

        envelope_volume[i] = 0;

        switch (psg_register[13 + i * 3] & 0xf) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 9:
            if (psg_envelope_counter[i] < psg_envelope_interval[i] * 32) {
                envelope_volume[i] = 31
                        - psg_envelope_counter[i] / psg_envelope_interval[i];
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else {
                envelope_volume[i] = 0;
            }
            break;
        case 4:
        case 5:
        case 6:
        case 7:
        case 15:
            if (psg_envelope_counter[i] < psg_envelope_interval[i] * 32) {
                envelope_volume[i] = psg_envelope_counter[i]
                        / psg_envelope_interval[i];
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else {
                envelope_volume[i] = 0;
            }
            break;
        case 8:
            if (psg_envelope_counter[i] < psg_envelope_interval[i] * 32) {
                envelope_volume[i] = 31
                        - psg_envelope_counter[i] / psg_envelope_interval[i];
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter[i] -= psg_envelope_interval[i] * 32;
                envelope_volume[i] = 31;
            }
            break;
        case 10:
            if (psg_envelope_counter[i] < psg_envelope_interval[i] * 32) {
                envelope_volume[i] = 31
                        - psg_envelope_counter[i] / psg_envelope_interval[i];
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter[i]
                    < psg_envelope_interval[i] * 64) {
                envelope_volume[i] = psg_envelope_counter[i]
                        / psg_envelope_interval[i] - 32;
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter[i] -= psg_envelope_interval[i] * 64;
                envelope_volume[i] = 31;
            }
            break;
        case 11:
            if (psg_envelope_counter[i] < psg_envelope_interval[i] * 32) {
                envelope_volume[i] = 31
                        - psg_envelope_counter[i] / psg_envelope_interval[i];
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else {
                envelope_volume[i] = 31;
            }
            break;
        case 12:
            if (psg_envelope_counter[i] < psg_envelope_interval[i] * 32) {
                envelope_volume[i] = psg_envelope_counter[i]
                        / psg_envelope_interval[i];
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter[i] -= psg_envelope_interval[i] * 32;
                envelope_volume[i] = 0;
            }
            break;
        case 13:
            if (psg_envelope_counter[i] < psg_envelope_interval[i] * 32) {
                envelope_volume[i] = psg_envelope_counter[i]
                        / psg_envelope_interval[i];
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else {
                envelope_volume[i] = 31;
            }
            break;
        case 14:
            if (psg_envelope_counter[i] < psg_envelope_interval[i] * 32) {
                envelope_volume[i] = psg_envelope_counter[i]
                        / psg_envelope_interval[i];
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else if (psg_envelope_counter[i]
                    < psg_envelope_interval[i] * 64) {
                envelope_volume[i] = 63
                        - psg_envelope_counter[i] / psg_envelope_interval[i];
                psg_envelope_counter[i] += SAMPLING_INTERVAL;
            } else {
                psg_envelope_counter[i] -= psg_envelope_interval[i] * 64;
                envelope_volume[i] = 0;
            }
            break;
        }

    }

    // Run Oscillator

    for (int i = 0; i < 3 * PSG_NUMBERS; i++) {
        pon_count = psg_osc_counter[i] += SAMPLING_INTERVAL;
        if (pon_count < (psg_osc_interval[i] / 2)) {
            tone_output[i] = psg_tone_on[i];
        } else if (pon_count > psg_osc_interval[i]) {
            psg_osc_counter[i] -= psg_osc_interval[i];
            tone_output[i] = psg_tone_on[i];
        } else {
            tone_output[i] = 0;
        }
    }

    // Mixer

    master_volume = 0;

    for (int i = 0; i < PSG_NUMBERS; i++) {
        for (int j = 0; j < 3; j++) {
            if ((tone_output[j + i * 3] + noise_output[j + i * 3]) > 0) {
                if ((psg_register[j + 8 + i * 16] & 0x10) == 0) {
                    master_volume += psg_volume[(psg_register[j + 8 + i * 16]
                            & 0xf) * 2 + 1];
                } else {
                    master_volume += psg_volume[envelope_volume[i]];
                }
            }
        }

    }

    psg_master_volume = master_volume / (3 * PSG_NUMBERS);

    if (psg_master_volume > 255)
        psg_master_volume = 255;
    SysTick->SR &= 0;

#ifdef PSG_DEBUG
    GPIO_WriteBit(GPIOC, GPIO_Pin_3, Bit_RESET);
#endif

}
