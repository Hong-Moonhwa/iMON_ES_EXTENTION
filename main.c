/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 14/10/17 8:29p $
 * @brief    NUC029 Series SPI Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "arm_math.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t LED1_R, LED1_G, LED1_B, Blink,brea=0,LED_cnt=0,brea_cnt=0;

/* Variables for DSP PID */
arm_pid_instance_f32 PIDS;

/* Variables for software PID */
float A0, A1, A2, state[3], Kp = 0.4, Ki = 0.4, Kd = 0, target, ival;

/* Variables for DSP and Software PID */
float output[100], ee;
uint8_t puRTxBuf[64]={0,};

float PID(float in)
{
    float out;

    A0 = Kp + Ki;
    A1 = -Kp - (2 * Kd);
    A2 = Kd;

    /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
    out = (A0 * in) + (A1 * state[0]) + (A2 * state[1]) + (state[2]);

    /* Update state */
    state[1] = state[0];
    state[0] = in;
    state[2] = out;

    /* return to application */
    return (out);
}

uint8_t g_fault_finderData[8]  = {0x01, 0x03, 
                                          0x00, 0x01, 
                                          0x00, 0x06, 
                                          0x94, 0x08};


/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);
void UART_Init(void);
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);
void SPI2_IRQHandler(void);

void TMR1_IRQHandler(void);

void SPI2_IRQHandler(void)
{
#if 0
    /* Check RX EMPTY flag */
    while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
    {
        /* Read RX FIFO */
        g_au32DestinationData[g_u32RxDataCount++] = SPI_READ_RX0(SPI0);
    }
    /* Check TX FULL flag and TX data count */
    while((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (g_u32TxDataCount < TEST_COUNT))
    {
        /* Write to TX FIFO */
        SPI_WRITE_TX0(SPI0, g_au32SourceData[g_u32TxDataCount++]);
    }
    if(g_u32TxDataCount >= TEST_COUNT)
        SPI_DisableInt(SPI0, SPI_FIFO_TX_INT_MASK); /* Disable TX FIFO threshold interrupt */

    /* Check the RX FIFO time-out interrupt flag */
    if(SPI_GetIntFlag(SPI0, SPI_FIFO_TIMEOUT_INT_MASK))
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while((SPI0->STATUS & SPI_STATUS_RX_EMPTY_Msk) == 0)
            g_au32DestinationData[g_u32RxDataCount++] = SPI_READ_RX0(SPI0);
    }
#endif
}


void TMR1_IRQHandler(void)
{
	  uint32_t LED_duty,RLED,BLED,GLED,LED_brea;
    if(TIMER_GetIntFlag(TIMER1) == 1) {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);
        LED_cnt++;
        LED_duty=LED_cnt%100;
        if((brea==1)&&((brea_cnt%2)==0))
            LED_brea=100-(LED_cnt/100);
        else if(brea==1)
            LED_brea=LED_cnt/100;
        else
            LED_brea=0;

        RLED=((int32_t)(LED1_R-LED_duty-LED_brea)>0)?1:0;
        BLED=((int32_t)(LED1_B-LED_duty-LED_brea)>0)?1:0;
        GLED=((int32_t)(LED1_G-LED_duty-LED_brea)>0)?1:0;

        if(LED_cnt>=(Blink*1000)) {
            RLED=0;
            BLED=0;
            GLED=0;
        }
		//printf("LED[0x%2x][0x%2x][0x%2x]\n ", RLED, BLED, GLED);

        
        //PA->DOUT = (PC->DOUT|BIT0|BIT1|BIT2|BIT3)&(~((RLED<<0)|(GLED<<1)|(BLED<<2)|(BLED<<3)));
        PA->DOUT = (PA->DOUT|BIT0|BIT1|BIT2|BIT3)&(~(RLED<<0));
;
        if(LED_cnt==10000) {
            LED_cnt=0;
            brea_cnt++;
        }
			
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Control  (Address Byte: Parity Bit =1 , Data Byte:Parity Bit =0)                        */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data)
{
    /* Set UART parity as MARK and ship baud rate setting */
    UART_SetLine_Config(UART0, 0, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* Send data */
    UART_WRITE(UART0, u8data);
}

void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    /* Set UART parity as SPACE and ship baud rate setting */
    UART_SetLine_Config(UART0, 0, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* Send data */
    UART_Write(UART0, pu8TxBuf, u32WriteBytes);
}
void RS485_ReadDataByte(uint8_t *puRTxBuf, uint32_t u32ReadBytes)
{
    /* Set UART parity as SPACE and ship baud rate setting */
    UART_SetLine_Config(UART0, 0, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* Send data */
    UART_Read(UART0, puRTxBuf, u32ReadBytes);
}



/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t i, CalTime;

    PIDS.Kp = 0.4;
    PIDS.Ki = 0.4;
    PIDS.Kd = 0;
    /* Target value*/
    target = 500;
    /* Inital value */
    ival = 0;
    /* Initial value and target value error */
    ee = target - ival;

    uint8_t u8InChar = 0xFF;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
  //  SYS_ResetModule(UART0_RST); 
    UART_Init();
#if 0 /* NuConsole without UART */ 
    NuConsole_Init(); 
#endif
    /* Init SPI */
    SPI_Init();
    
    printf("\n\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("|                   iMON Communication Code                    |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");
    
   /* Init TIMER0 for performance comparing */
    TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1);

    TIMER_Start(TIMER0);

    /************************* DSP PID ******************************/
    /* Initial DSP PID controller function*/
  //  arm_pid_init_f32(&PIDS,0);

    /* Calculate PID controller function 100 times*/
    for(i = 1; i < 100; i++)
    {
        output[i] = arm_pid_f32(&PIDS,ee);
        ee = target-output[i-1];
    }

    TIMER_Close(TIMER0);
    CalTime = TIMER_GetCounter(TIMER0);
    printf("\nDSP PID: It took %d HXT clocks\n", CalTime);

    /********************** Software PID ****************************/
    /* Re-Initialization TIMER0 for performance comparing */
    TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1);

    TIMER_Start(TIMER0);

    /* Calculate PID controller function 100 times*/
    for(i = 1; i < 100; i++)
    {
        output[i] = PID(ee);
        ee = target-output[i-1];
    }

    TIMER_Close(TIMER0);
    CalTime = TIMER_GetCounter(TIMER0);
    printf("Software PID: It took %d HXT clocks\n", CalTime);
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    
    
    /* Set RS485-Master as AUD mode */
    /* Enable AUD mode to HW control RTS pin automatically */
    /* You also can use GPIO to control RTS pin for replacing AUD mode*/

       /* Set RTS pin active level as high level active */
  //  UART0->MFPL &= ~UART_MODEM_RTS_Msk;
  //  UART0->MFPL |= UART_RTS_IS_HIGH_LEV_ACTIVE;
#if 0
    for(i=0; i<8 ; i++)
    {
      u8InChar = g_fault_finderData[i];
      UART_WRITE(UART0, u8InChar);
    }
#endif


#if 1
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT); //RUN BLED
    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT); //ST1 GLED
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);  //ST2 RLED
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);  //ST3 RLED

	GPIO_SetMode(PB, BIT0, GPIO_MODE_QUASI); //SW_IN1
	GPIO_SetMode(PB, BIT1, GPIO_MODE_QUASI); //
	GPIO_SetMode(PB, BIT2, GPIO_MODE_QUASI);  //
	GPIO_SetMode(PB, BIT3, GPIO_MODE_QUASI);  //
	GPIO_SetMode(PB, BIT4, GPIO_MODE_QUASI); //
	GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI); 
	GPIO_SetMode(PB, BIT6, GPIO_MODE_QUASI);  //
	GPIO_SetMode(PB, BIT7, GPIO_MODE_QUASI);  //SW_IN8


	GPIO_SetMode(PD, BIT15, GPIO_MODE_INPUT);  // DIN1
	GPIO_SetMode(PD, BIT14, GPIO_MODE_INPUT); 	// DIN2
	GPIO_SetMode(PD, BIT13, GPIO_MODE_INPUT); 	// DIN3
	GPIO_SetMode(PD, BIT12, GPIO_MODE_INPUT);  // DIN4

	GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);  // DOUT_EN
	GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT); 	// DOUT_EX1
	GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT); 	// DOUT_EX2
	GPIO_SetMode(PC, BIT1, GPIO_MODE_OUTPUT);  // DOUT_EX3
	GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);  // DOUT_EX4




	GPIO_SetMode(PA, BIT12, GPIO_MODE_OUTPUT);  // PT100_EN
	GPIO_SetMode(PA, BIT13, GPIO_MODE_OUTPUT); 	// S0
	GPIO_SetMode(PA, BIT14, GPIO_MODE_OUTPUT); 	// S1
	GPIO_SetMode(PA, BIT15, GPIO_MODE_OUTPUT);  // S2



	printf("\nSW_IN1[%x]",PB0);//->PIN);
	printf("\nSW_IN2[%x]",PB1);//->PIN << 1);
	printf("\nSW_IN3[%x]",PB2);//->PIN << 2);
	printf("\nSW_IN4[%x]",PB3);//->PIN << 3);
	printf("\nSW_IN5[%x]",PB4);//->IN << 4);
	printf("\nSW_IN6[%x]",PB5);//->PIN << 5);
	printf("\nSW_IN7[%x]",PB6);//->PIN << 6);
	printf("\nSW_IN8[%x]\n\n",PB7);//->PIN << 7);						




	/* Enable peripheral clock */
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 10000);
    TIMER_EnableInt(TIMER1);
    /* Enable Timer1 NVIC */
    NVIC_EnableIRQ(TMR1_IRQn);

    /*setting RGB LED*/
    LED1_R=100;
    LED1_G=100;
    LED1_B=100;
    Blink=10;
    brea=1;

    TIMER_Start(TIMER1);


#endif

#if 1
    RS485_SendDataByte(g_fault_finderData, 8);
    RS485_ReadDataByte(puRTxBuf,17);
   
   for(i=0;i<17;i++)
   {
		printf("485 RECEIVE [%2x]\n", puRTxBuf[i]);
   }


    /* Set TX FIFO threshold and enable FIFO mode. */
//    SPI_EnableFIFO(SPI2, 4, 4);
	PA->DOUT = (PA->DOUT|BIT0|BIT1|BIT2|BIT3)&(~((1<<1)|(1<<2)|(1<<3)));
	PA1 = 1;
	PA2 = 1;
	PA3 = 1;
    u8InChar = 0xFF; 
    while(1)
    {

#if 1
					RS485_ReadDataByte(puRTxBuf,17);
		
					for(i=0;i<17;i++)
					{
						 printf("485 [%2x]\n ", puRTxBuf[i]);
					}


		if(UART_GET_RX_EMPTY(UART0)==0)
		{
			UART_SetLine_Config(UART0, 0, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);
            printf("485 RECEIVE [%x]\n ", u8InChar);		

//			RS485_ReadDataByte(puRTxBuf,64);

	//		for(i=0;i<64;i++)
		//	{
			//	 printf("485 RECEIVE [%x]\n ", puRTxBuf[i]);
		//	}

		}
#else
        /* Check if buffer full */
        if(UART_IS_RX_READY(UART0))
        {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);
            printf("485 RECEIVE [0x%2x]\n ", u8InChar);
        }
#endif
   		/* Check RX EMPTY flag */
   		if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI2) == 0)
   		{
   			printf("SPI RECEIVE [0x%2x]\n ", SPI_READ_RX(SPI2)); /* Read RX FIFO */
	   	}

		
   }
   
#endif

//    while(1);  

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    
    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    
        /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();
    
    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);;

    /* Set core clock as PLL_CLOCK from PLL */
   // CLK_SetCoreClock(PLL_CLOCK);
 
      /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);  
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(SPI2_MODULE);
    
    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_HXT, MODULE_NoMsk);
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
 	/* LED LED_RUN ~ ST3 */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk |SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_GPIO | SYS_GPA_MFPL_PA1MFP_GPIO | SYS_GPA_MFPL_PA2MFP_GPIO |SYS_GPA_MFPL_PA3MFP_GPIO);	

	/* DIN1 ~ DIN 4 */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD12MFP_Msk | SYS_GPD_MFPH_PD13MFP_Msk | SYS_GPD_MFPH_PD14MFP_Msk |SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD12MFP_GPIO | SYS_GPD_MFPH_PD13MFP_GPIO | SYS_GPD_MFPH_PD14MFP_GPIO |SYS_GPD_MFPH_PD15MFP_GPIO);	

	/* DOUT DOUT_EN ~ DOUT_EX4  */
	SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk |SYS_GPC_MFPL_PC3MFP_Msk |SYS_GPC_MFPL_PC4MFP_Msk);
	SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_GPIO | SYS_GPC_MFPL_PC1MFP_GPIO | SYS_GPC_MFPL_PC2MFP_GPIO |SYS_GPC_MFPL_PC3MFP_GPIO |SYS_GPC_MFPL_PC4MFP_GPIO);	 

	/* DIP SWITCH SW_IN1 ~ SW_IN8 */
	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk| SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk |SYS_GPB_MFPL_PB3MFP_Msk \
					|SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk |SYS_GPB_MFPL_PB7MFP_Msk);
	SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_GPIO | SYS_GPB_MFPL_PB1MFP_GPIO | SYS_GPB_MFPL_PB2MFP_GPIO |SYS_GPB_MFPL_PB3MFP_GPIO \
					|SYS_GPB_MFPL_PB4MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO | SYS_GPB_MFPL_PB6MFP_GPIO |SYS_GPB_MFPL_PB7MFP_GPIO);	 


	/* PT EN, PT100_S0 ~ S2 */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk |SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_GPIO | SYS_GPA_MFPH_PA13MFP_GPIO | SYS_GPA_MFPH_PA14MFP_GPIO |SYS_GPA_MFPH_PA15MFP_GPIO);	


	/* Tempture : I2C1   */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE5MFP_Msk | SYS_GPE_MFPL_PE4MFP_Msk);
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE5MFP_I2C1_SDA | SYS_GPE_MFPL_PE4MFP_I2C1_SCL);	
    /* I2C1 ALRT */
	SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC7MFP_Msk );
	SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC7MFP_I2C1_SMBSUS );	  
	/* I2C0 Master with iMON : I2C0   */
    SYS->GPE_MFPL &= ~(SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD5MFP_Msk);
    SYS->GPE_MFPL |= (SYS_GPD_MFPL_PD4MFP_I2C0_SDA | SYS_GPD_MFPL_PD5MFP_I2C0_SCL);	



	/* Fault Finder UART0 : Set PD multi-function pins for UART0 RXD and TXD  */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);
	SYS->GPA_MFPL = SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk) |SYS_GPA_MFPL_PA3MFP_UART0_nRTS;   
    /*  Debug UART UART1: Set P1 multi-function pins for UART1 RXD and TXD */
    SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE9MFP_Msk | SYS_GPE_MFPH_PE8MFP_Msk);
    SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE9MFP_UART1_RXD | SYS_GPE_MFPH_PE8MFP_UART1_TXD);
    /*  TeMP UART UART3: Set P1 multi-function pins for UART1 RXD and TXD */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA8MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA8MFP_UART3_TXD | SYS_GPA_MFPH_PA9MFP_UART3_RXD);	
  
    /* Setup SPI2 multi-function pins */
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC13MFP_Msk | SYS_GPC_MFPH_PC10MFP_Msk | SYS_GPC_MFPH_PC11MFP_Msk | SYS_GPC_MFPH_PC12MFP_Msk);
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC13MFP_SPI2_SS | SYS_GPC_MFPH_PC10MFP_SPI2_MOSI | SYS_GPC_MFPH_PC11MFP_SPI2_MISO | SYS_GPC_MFPH_PC12MFP_SPI2_CLK);


	/* JTAG ICE  */
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF5MFP_Msk | SYS_GPF_MFPL_PF6MFP_Msk);
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF5MFP_ICE_CLK | SYS_GPF_MFPL_PF6MFP_ICE_DAT);	

	/* XT1_IN 12M_IN / OUR */
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF3MFP_Msk | SYS_GPF_MFPL_PF4MFP_Msk);
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF3MFP_XT1_OUT | SYS_GPF_MFPL_PF4MFP_XT1_IN);		

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void SPI_Init(void)
{
#if 1
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI0 as a low level active device. */
    SPI_Open(SPI2, SPI_SLAVE, SPI_MODE_0, 32, 0);
    
#else // Master 
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);
#endif
    
}
void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    UART_Open(UART0, 9600);
    /* Configure UART1: 115200, 8-bit word, no parity bit, 1 stop bit. */


    UART_Open(UART1, 115200);

    UART_SelectRS485Mode(UART0, UART_ALTCTL_RS485AUD_Msk, 0);
	UART0->MODEM &= ~UART_MODEM_RTSACTLV_Msk;
	UART0->MODEM |= UART_RTS_IS_HIGH_LEV_ACTIVE;

}
/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

