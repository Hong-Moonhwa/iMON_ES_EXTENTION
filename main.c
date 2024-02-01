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
#include "ADS1115.h"




static uint8_t g_comm_485_status = comm_485_ready;
static uint8_t g_oil_level_status = oil_level_ready;
static uint8_t g_i2c_sensor_status = i2c_sensor_normal;



/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t LED1_R, LED1_G, LED1_B, Blink,brea=0,LED_cnt=0,brea_cnt=0, TMR0_cnt=0, TEMP_Cnt=0, TEMP_save=0;

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
};


uint8_t g_fault_finderData[8]  = {0x0,};
uint8_t g_fault_RecvData[32]  = {0,};
uint8_t g_iMON_RecvData[64]  = {0,};

volatile uint32_t g_u32_485RxDataCount=0;
volatile uint32_t g_u32_spiRxDataCount;

volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_485_flags=0;



static I2C_FUNC s_I2C0HandlerFn = NULL;
static I2C_FUNC s_I2C1HandlerFn = NULL;



/*---------------------------------------------------------------------------------------------------------*/
/* Global variables I2C Slave                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
static uint32_t slave_buff_addr;
static uint8_t g_au8SlvData[64]={0x0,};
static uint8_t g_au8SlvRxData[64]={0x0,};
static volatile uint8_t g_u8SlvTRxAbortFlag = 0;
static volatile uint8_t g_u8SlvTimeoutFlag = 0;
volatile uint8_t g_u8SlvDeviceAddr;
volatile uint8_t g_u8SlvDataLen;



static uint8_t g_au8SlvCounter = 0;

uint8_t static countSlv_i =0;

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables I2C Master                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
static  uint8_t g_u8MstDeviceAddr;
static  uint8_t g_au8MstTxData[32]={0x0,};
static  uint8_t g_u8MstRxData[32]={0x0,};
static  uint8_t g_u8MstDataLen;
static  uint8_t g_u8MstEndFlag = 0;


static  uint8_t g_u8MstTxAbortFlag = 0;
static  uint8_t g_u8MstRxAbortFlag = 0;
static  uint8_t g_u8MstReStartFlag = 0;
static  uint8_t g_u8MstTimeoutFlag = 0;








static  uint8_t g_testCount50_flag = 2;
static uint8_t g_tempture_value[8]={0,};


void set_i2sensor_status(uint8_t set_status)
{
	g_i2c_sensor_status = set_status;
}

uint8_t get_i2sensor_status()
{
	return	g_i2c_sensor_status;

}


void set_oillevel_status(uint8_t set_status)
{
	g_oil_level_status = set_status;
}

uint8_t get_oillevel_status()
{
	return	g_oil_level_status;

}



void set_485comm_status(uint8_t set_status)
{
	g_comm_485_status = set_status;
}

uint8_t get_485comm_status()
{
	return	g_comm_485_status;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;
	
    u32Status = I2C_GET_STATUS(I2C0);
	//printf("\n[Slave] I2C0_IRQHandler Proccesed\n");

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
        g_u8SlvTimeoutFlag = 1;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 capture trigger interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
		TMR0_cnt++;
		TEMP_Cnt = TMR0_cnt;


		if(TMR0_cnt> 1000000 * 50)
		{
       		g_testCount50_flag=1;
			if(TMR0_cnt> 1000000 * 60)
			{
       			g_testCount50_flag=2;
				TMR0_cnt=0;
				printf("Running Value time\n");
			}
			//printf("Clean Value time\n");
		}

		

    }


}


void TMR1_IRQHandler(void)
{
	  uint32_t LED_duty,RLED,BLED,GLED,LED_brea;
    if(TIMER_GetIntFlag(TIMER1) == 1) 
	{
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

        
        //PA->DOUT = (PA->DOUT|BIT0|BIT1|BIT2|BIT3)&(~((RLED<<0)|(GLED<<1)|(BLED<<2)|(BLED<<3)));
       // PA->DOUT = (PA->DOUT|BIT0|BIT1|BIT2|BIT3)&(~(RLED<<0));
		PA->DOUT = (PA->DOUT|BIT0)&(~(RLED<<0));


        if(LED_cnt==3000) {
            LED_cnt=0;
            brea_cnt++;
        }
			
    }
}


void UART0_IRQHandler(void)
{

	int i,j,k;

	UART_SetLine_Config(UART0, 0, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
	
	/* Check RX EMPTY flag */
	while(UART_GET_RX_EMPTY(UART0)==0)
	{

		/* Read RX FIFO */
		g_fault_RecvData[g_u32_485RxDataCount++] = UART_READ(UART0);
		g_485_flags=1;
			
	}


}


void SPI2_IRQHandler(void)
{
#if 0
	/* Check RX EMPTY flag */
	if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI2) == 0)
	{
		printf("SPI RECEIVE [0x%2x]\n ", SPI_READ_RX(SPI2)); /* Read RX FIFO */
	}
#endif
#if 0

    /* Check TX FULL flag and TX data count */
    while((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (g_u32TxDataCount < TEST_COUNT))
    {
        /* Write to TX FIFO */
        SPI_WRITE_TX0(SPI0, g_au32SourceData[g_u32TxDataCount++]);
    }
    if(g_u32TxDataCount >= TEST_COUNT)
        SPI_DisableInt(SPI0, SPI_FIFO_TX_INT_MASK); /* Disable TX FIFO threshold interrupt */
    /* Check RX EMPTY flag */
    while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
    {
        /* Read RX FIFO */
        g_au32DestinationData[g_u32RxDataCount++] = SPI_READ_RX0(SPI0);
    }

    /* Check the RX FIFO time-out interrupt flag */
    if(SPI_GetIntFlag(SPI0, SPI_FIFO_TIMEOUT_INT_MASK))
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while((SPI0->STATUS & SPI_STATUS_RX_EMPTY_Msk) == 0)
            g_au32DestinationData[g_u32RxDataCount++] = SPI_READ_RX0(SPI0);
    }
#endif
}

int I2C_Reeceived_valid()
{

	if(g_au8SlvRxData[0] !=0xff)
	{
			printf("\nI2C_Slave iMON Received Packet Mismatch. [0][%x]\n",g_au8SlvRxData[0] );
			return 1;
	}
	if(g_au8SlvRxData[1] !=0x01)
	{
			printf("\nI2C_Slave iMON Received Packet Mismatch. [1][%x]\n",g_au8SlvRxData[1] );
			return 1;
	}
	if(g_au8SlvRxData[2] !=0x01)
	{
			printf("\nI2C_Slave iMON Received Packet Mismatch. [2][%x]\n",g_au8SlvRxData[2] );
			return 1;
	}
	if(g_au8SlvRxData[3] !=0x00)
	{
			printf("\nI2C_Slave iMON Received Packet Mismatch. [3][%x]\n",g_au8SlvRxData[3] );
			return 1;

	}
	if(g_au8SlvRxData[4] !=0x00)
	{
			printf("\nI2C_Slave iMON Received Packet Mismatch. [4][%x]\n",g_au8SlvRxData[4] );
			return 1;

	}
	if(g_au8SlvRxData[5] !=0xf8)
	{
			printf("\nI2C_Slave iMON Received Packet Mismatch. [5][%x]\n",g_au8SlvRxData[5] );
			return 1;

	}
	//printf("\nI2C_Slave iMON Received Packet Valid.\n");

	return 0;
}

void set_CRC16_faultfinder()
{

	uint8_t pin_value =0x0;
	uint16_t crc16_result = 0x0;

	pin_value = get_PinValue();
	g_fault_finderData[0] = pin_value;
	g_fault_finderData[1] = 0x03;
	g_fault_finderData[2] = 0x00;
	g_fault_finderData[3] = 0x01;	

	g_fault_finderData[4] = 0x00;
	g_fault_finderData[5] = 0x06;



	crc16_result = ModBus_CRC16(g_fault_finderData,6);

    g_fault_finderData[6] = crc16_result & 0xff;  
    g_fault_finderData[7] = (crc16_result >> 8) & 0xff;

	printf("\nCRC16 VALUE [0x%02x][0x%02x]",g_fault_finderData[6],g_fault_finderData[7]);


}


int I2C_Transmit_made_test()
{
	static uint8_t toggle_upcnt = 0,toggle_downcnt = 1, toggle_value = 1, err_flag =0;
	
		 g_au8SlvData[0] = 0xff;
		 g_au8SlvData[1] = get_PinValue();
	     g_au8SlvData[2] = 0x00;
		 g_au8SlvData[3] = 0x1e;	 

		if(g_testCount50_flag==1)
		{
			 g_au8SlvData[4] = 0; /* STOP */
			 g_au8SlvData[5] = 0;
			 g_au8SlvData[6] = 0;



		}
		else if(g_testCount50_flag==2)
		{
			toggle_upcnt = ~toggle_upcnt & 0x01;
			toggle_downcnt  = ~toggle_downcnt & 0x01;
			 g_au8SlvData[4] = 1; /* RUN */
			 g_au8SlvData[5] = toggle_upcnt;
			 g_au8SlvData[6] = toggle_downcnt;

	
		}		 





		if((0x1  & ~PB0) == 1)
		{
		
			g_au8SlvData[7] = 0x01; /* Occurred Error */
			
			g_au8SlvData[8] = 0x00;
			g_au8SlvData[9] = toggle_value;
			

			err_flag = 1 ;
			printf("Occurred Error\n");
		}
		else
		{
			 g_au8SlvData[7] = 0x00;
			 g_au8SlvData[8] = 0x00;
			 g_au8SlvData[9] = 0x00;

			 if(err_flag == 1)
			 {
			 	toggle_value++;
				if(toggle_value==60)
				{
				   toggle_value =1;
				}
				err_flag = 0;
			 }
		}



		if(g_testCount50_flag==1)
		{
			 g_au8SlvData[10] = 0x1; /* Oil level Error : 0, Normal : 1 */
			 g_testCount50_flag = 0;
			 

		}
		else if(g_testCount50_flag==2)
		{
			 g_au8SlvData[10] = 0x0; /* Oil level Error : 0, Normal : 1 */
			 g_testCount50_flag = 0;
		}

		 g_au8SlvData[11] = 0x00;   /* Tempeture */
		 g_au8SlvData[12] = g_tempture_value[0]; 
		 
		 g_au8SlvData[13] = 0x00;   /* Tempeture */
		 g_au8SlvData[14] = g_tempture_value[0]+75;   
		 
		 g_au8SlvData[15] = 0x00;   /* Tempeture */		
		 g_au8SlvData[16] = g_tempture_value[0]+85; 
		 
		 g_au8SlvData[17] = 0x00;   /* Tempeture */
		 g_au8SlvData[18] = g_tempture_value[0]+80;   
		 
		 g_au8SlvData[19] = 0x0;  /* Temp Tempeture */
		 g_au8SlvData[20] = 0x0;
		 
		 g_au8SlvData[21] = 0x0;   /* Temp Tempeture */
		 g_au8SlvData[22] = 0x0;
		 
		 g_au8SlvData[23] = 0x0;  /* Temp Tempeture */
		 g_au8SlvData[24] = 0x0;
		 
		 g_au8SlvData[25] = 0x0;    /* Temp Tempeture */	
		 g_au8SlvData[26] = 0x0;
		 
		 g_au8SlvData[27] = 0x00;  /* Reserved */
		 g_au8SlvData[28] = 0x00;  /* Reserved */
		 g_au8SlvData[29] = 0x00;  /* Reserved */
		 g_au8SlvData[30] = 0x00;  /* Reserved */
		 g_au8SlvData[31] = EXTEN_BD_TYPE;  /* Extend Board Type */
		 
		 g_au8SlvData[32] = 0x01; /* Board Major Version */
		 g_au8SlvData[33] = 0x00; /* Board Minor Version */
		 
		 g_au8SlvData[34] = 0xf8;  /* EXT */
		 
		 
}



void get_TemptureWarningCheck()
{
	 if(g_tempture_value[0] > 110)
	 {
		g_au8SlvData[7] = 0x01;
		g_au8SlvData[9] = 0x47;
	 }
	
	 if(g_au8SlvData[7] == 0x01 && g_au8SlvData[9] == 0x47)
	 {
		 if(g_tempture_value[0] <= 110)
		{
			g_au8SlvData[7] = 0x00;
			g_au8SlvData[9] = 0x00;
		}
	 }
	 
	 if( g_tempture_value[1] > 95)
	 {
		 g_au8SlvData[7] = 0x01;
		 g_au8SlvData[9] = 0x48;
	 }	
	
	 if(g_au8SlvData[7] == 0x01 && g_au8SlvData[9] == 0x48)
	 {
		 if(g_tempture_value[1] <= 95)
		{
			g_au8SlvData[7] = 0x00;
			g_au8SlvData[9] = 0x00;
		}
	 }
	
	 
	 if( g_tempture_value[2] > 95)
	 {
		g_au8SlvData[7] = 0x01;
		g_au8SlvData[9] = 0x48;
	
	 }
	
	 if(g_au8SlvData[7] == 0x01 && g_au8SlvData[9] == 0x48)
	 {
		 if(g_tempture_value[2] <= 95)
		{
			g_au8SlvData[7] = 0x00;
			g_au8SlvData[9] = 0x00;
		}
	 }
	
	 
	 if( g_tempture_value[3] > 100)
	 {
		g_au8SlvData[7] = 0x01;
		g_au8SlvData[9] = 0x49;
	 }
	
	if(g_au8SlvData[7] == 0x01 && g_au8SlvData[9] == 0x49)
	{
		if(g_tempture_value[3] <= 100)
		{
		   g_au8SlvData[7] = 0x00;
		   g_au8SlvData[9] = 0x00;
		}
	}


}



int I2C_Transmit_clean()
{
	
		 g_au8SlvData[0] = 0xff;
		 g_au8SlvData[1] = get_PinValue();
	     g_au8SlvData[2] = 0x00;
		 g_au8SlvData[3] = 0x1e;	 
		 g_au8SlvData[4] = 0x00;
	     g_au8SlvData[5] = 0x00;
		 g_au8SlvData[6] = 0x00;
	     g_au8SlvData[7] = 0x01;


		 g_au8SlvData[8] = 0x00;
		 g_au8SlvData[9] = 0x3C;

		if(get_PinOilLevelValue() == 1)
		{
			g_au8SlvData[10] = get_oillevel_status();
		}
		else
		{
			g_au8SlvData[10] = 0xff;
		}	 
		 g_au8SlvData[11] = 0x00;   /* Tempeture */
		 g_au8SlvData[12] = g_tempture_value[0]; 
		 
		 g_au8SlvData[13] = 0x00;   /* Tempeture */
		 g_au8SlvData[14] = g_tempture_value[1];   
		 
		 g_au8SlvData[15] = 0x00;   /* Tempeture */		
		 g_au8SlvData[16] = g_tempture_value[2]; 
		 
		 g_au8SlvData[17] = 0x00;   /* Tempeture */
		 g_au8SlvData[18] = g_tempture_value[3];   
		 
		 g_au8SlvData[19] = 0x0;  /* Temp Tempeture */
		 g_au8SlvData[20] = 0xff;//g_tempture_value[4];
		 
		 g_au8SlvData[21] = 0x0;   /* Temp Tempeture */
		 g_au8SlvData[22] = 0xff;//g_tempture_value[5];
		 
		 g_au8SlvData[23] = 0x0;  /* Temp Tempeture */
		 g_au8SlvData[24] = 0xff;//g_tempture_value[6];
		 
		 g_au8SlvData[25] = 0x0;    /* Temp Tempeture */	
		 g_au8SlvData[26] = 0xff;//g_tempture_value[7];
		 
		 g_au8SlvData[27] = 0x00;  /* Reserved */
		 g_au8SlvData[28] = 0x00;  /* Reserved */
		 g_au8SlvData[29] = 0x00;  /* Reserved */
		 g_au8SlvData[30] = 0x00;  /* Reserved */
		 g_au8SlvData[31] = EXTEN_BD_TYPE;  /* Extend Board Type */
		 
		 g_au8SlvData[32] = 0x01; /* Board Major Version */
		 g_au8SlvData[33] = 0x00; /* Board Minor Version */
		 
		 g_au8SlvData[34] = 0xf8;  /* EXT */

	//	 printf("OIL[0x%x]T1[0x%x]T2[0x%x]T3[0x%x]T4[0x%x]",g_au8SlvData[10],\
		 													g_au8SlvData[12],\
		 													g_au8SlvData[14],\
		 													g_au8SlvData[16],\
		 													g_au8SlvData[18]);
//		get_TemptureWarningCheck(); 
}

int I2C_Transmit_made()
{
	
		 g_au8SlvData[0] = 0xff;
		 g_au8SlvData[1] = get_PinValue();
	     g_au8SlvData[2] = 0x00;
		 g_au8SlvData[3] = 0x1e;	 
		 g_au8SlvData[4] = g_fault_RecvData[4];
	     g_au8SlvData[5] = g_fault_RecvData[6];
		 g_au8SlvData[6] = g_fault_RecvData[8];
	     g_au8SlvData[7] = g_fault_RecvData[12];


		 g_au8SlvData[8] = 0x00;
		 g_au8SlvData[9] = g_fault_RecvData[14];

		if(get_PinOilLevelValue() == 1)
		{
			g_au8SlvData[10] = get_oillevel_status();
		}
		else
		{
			g_au8SlvData[10] = 0xff;
		}
		 g_au8SlvData[11] = 0x00;   /* Tempeture */
		 g_au8SlvData[12] = g_tempture_value[0]; 
		 
		 g_au8SlvData[13] = 0x00;   /* Tempeture */
		 g_au8SlvData[14] = g_tempture_value[1];   
		 
		 g_au8SlvData[15] = 0x00;   /* Tempeture */		
		 g_au8SlvData[16] = g_tempture_value[2]; 
		 
		 g_au8SlvData[17] = 0x00;   /* Tempeture */
		 g_au8SlvData[18] = g_tempture_value[3];   
		 
		 g_au8SlvData[19] = 0x0;  /* Temp Tempeture */
		 g_au8SlvData[20] = 0xff;//g_tempture_value[4];
		 
		 g_au8SlvData[21] = 0x0;   /* Temp Tempeture */
		 g_au8SlvData[22] = 0xff;//g_tempture_value[5];
		 
		 g_au8SlvData[23] = 0x0;  /* Temp Tempeture */
		 g_au8SlvData[24] = 0xff;//g_tempture_value[6];
		 
		 g_au8SlvData[25] = 0x0;    /* Temp Tempeture */	
		 g_au8SlvData[26] = 0xff;//g_tempture_value[7];
		 
		 g_au8SlvData[27] = 0x00;  /* Reserved */
		 g_au8SlvData[28] = 0x00;  /* Reserved */
		 g_au8SlvData[29] = 0x00;  /* Reserved */
		 g_au8SlvData[30] = 0x00;  /* Reserved */
		 g_au8SlvData[31] = EXTEN_BD_TYPE;  /* Extend Board Type */
		 
		 g_au8SlvData[32] = 0x01; /* Board Major Version */
		 g_au8SlvData[33] = 0x00; /* Board Minor Version */
		 
		 g_au8SlvData[34] = 0xf8;  /* EXT */
		 
		//printf("OIL[0x%x]T1[0x%x]T2[0x%x]T3[0x%x]T4[0x%x]",g_au8SlvData[10],\
												   g_au8SlvData[12],\
												   g_au8SlvData[14],\
												   g_au8SlvData[16],\
												   g_au8SlvData[18]);

//		get_TemptureWarningCheck();


		 
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function            

	I2C_CTL_STA_SI                  
	I2C_CTL_STA_SI_AA       
	I2C_CTL_STO_SI            
	I2C_CTL_STO_SI_AA   
	I2C_CTL_SI               
	I2C_CTL_SI_AA             
	I2C_CTL_STA              
	I2C_CTL_STO              
	I2C_CTL_AA               
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{

	//printf("\niMON I2C0 GET RAW Data 0x%x\n",(unsigned char) I2C_GET_DATA(I2C0));
	//printf("\nI2C_SlaveTRx Status 0x%x\n",u32Status);


    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
		
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
		g_au8SlvRxData[g_u8SlvDataLen] = (unsigned char) I2C_GET_DATA(I2C0);

       	printf("\niMON I2C0 GET Data 0x%02x\n",g_au8SlvRxData[g_u8SlvDataLen]);
        g_u8SlvDataLen++;
		

		if(g_u8SlvDataLen == 6)
        {
        	slave_buff_addr=0;

            g_u8SlvDataLen = 0;
			countSlv_i = 0;

		}
 
      I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);

    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
			
		g_au8SlvCounter =0 ;

	    if(1)//I2C_Reeceived_valid()==0)
		{
    	       	I2C_SET_DATA(I2C0, g_au8SlvData[g_au8SlvCounter]);
    		    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
		     //	printf("\niMON I2C0 Sending Data 0x%02x\n",g_au8SlvData[g_au8SlvCounter]);
				g_au8SlvCounter++;
	    }
		else
		{ 
		    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
			g_au8SlvCounter=0;
		}
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
		printf("iMON Send OK\n");
#if 0
		for(int i=0; i<35;  i++)
		{
			printf("%02x ",g_au8SlvData[i]);
		}
#endif		
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
    	g_au8SlvCounter=0;
	
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    }
 	else if(u32Status == 0xB8)                  /* Own SLA+R has been receive; ACK has been return */
    {
			
			if(g_au8SlvCounter==35)
			{
			
				g_au8SlvCounter=0;
			    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
			}
    		else
    		{

		       	I2C_SET_DATA(I2C0, g_au8SlvData[g_au8SlvCounter]);
    		    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
				g_au8SlvCounter++;

    		}

	


 	}   
    else
    {
        printf("[SlaveTRx] Status [0x%x] Unexpected abort!!\n", u32Status);
        if(u32Status == 0x68)               /* Slave receive arbitration lost, clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else if(u32Status == 0xB0)          /* Address transmit arbitration lost, clear SI  */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else                                /* Slave bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        g_u8SlvTRxAbortFlag = 1;

    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler   Master                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;


    u32Status = I2C_GET_STATUS(I2C1);
//	printf("\n[Master] I2C1_IRQHandler Status 0x%x\n",u32Status);

    if(I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
        g_u8MstTimeoutFlag = 1;

    }
    else
    {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
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
    //CLK_DisablePLL();
    
    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);


    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);;

    /* Set core clock as PLL_CLOCK from PLL */
	CLK_SetCoreClock(PLL_CLOCK);

 
      /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);  
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(SPI2_MODULE);
	CLK_EnableModuleClock(I2C0_MODULE);
	CLK_EnableModuleClock(I2C1_MODULE);

    
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


	  
	/* I2C0 Master with iMON : I2C0   */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD5MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD4MFP_I2C0_SDA | SYS_GPD_MFPL_PD5MFP_I2C0_SCL);	

	/* Tempture : I2C1   */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE5MFP_Msk | SYS_GPE_MFPL_PE4MFP_Msk);
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE5MFP_I2C1_SDA | SYS_GPE_MFPL_PE4MFP_I2C1_SCL);	
    /* I2C1 ALRT */
	//SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC7MFP_Msk );
	//SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC7MFP_I2C1_SMBSUS );
    /* I2C pins enable schmitt trigger */
    //PA->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk);///???



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

	
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT); //RUN BLED
    GPIO_SetMode(PA, BIT1, GPIO_MODE_QUASI); //ST1 GLED
    GPIO_SetMode(PA, BIT2, GPIO_MODE_QUASI);  //ST2 RLED
    GPIO_SetMode(PA, BIT3, GPIO_MODE_QUASI);  //ST3 RLED

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




	GPIO_SetMode(PA, BIT12, GPIO_MODE_OUTPUT );  // PT100_EN
	GPIO_SetMode(PA, BIT13, GPIO_MODE_OUTPUT ); 	// S0
	GPIO_SetMode(PA, BIT14, GPIO_MODE_OUTPUT ); 	// S1
	GPIO_SetMode(PA, BIT15, GPIO_MODE_OUTPUT );  // S2 GPIO_MODE_QUASI GPIO_MODE_OPEN_DRAIN GPIO_MODE_OUTPUT
	PA12 = 1;/* EN */
	PA13 = 0;
	PA14 = 0;
	PA15 = 0;


	

}
void Timer_Init(void)
{
	 uint32_t i, CalTime;

	
	 /* Init TIMER0 for performance comparing */
	  TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1);
	
	  TIMER_Start(TIMER0);
	
	  /************************* DSP PID ******************************/
	  /* Initial DSP PID controller function*/
	//	arm_pid_init_f32(&PIDS,0);
	
	  /* Calculate PID controller function 100 times*/
	  for(i = 1; i < 100; i++)
	  {
		  output[i] = arm_pid_f32(&PIDS,ee);
		  ee = target-output[i-1];
	  }
	
	  TIMER_Close(TIMER0);
	  CalTime = TIMER_GetCounter(TIMER0);
	  printf("\nDSP PID: It took %d HXT clocks\n", CalTime);
	  printf("\n\nCPU @ %dHz\n", SystemCoreClock);	


	  /* Enable peripheral clock */
	  CLK_EnableModuleClock(TMR0_MODULE);
	  CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);	
      /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
	  TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);  
      TIMER_EnableInt(TIMER0);
      /* Enable Timer0 ~ Timer3 NVIC */
      NVIC_EnableIRQ(TMR0_IRQn);
	  TIMER_Start(TIMER0);

	
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

}

void SPI_Init(void)
{
#if 0
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI0 as a low level active device. */
    SPI_Open(SPI2, SPI_SLAVE, SPI_MODE_0, 32, 0);
//    SPI_EnableInt(SPI2,SPI_FIFO_TXTH_INT_MASK );
//	NVIC_EnableIRQ(SPI2_IRQn);
#else // Master 
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI2, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI2, SPI_SS, SPI_SS_ACTIVE_LOW);
#endif
    
}
void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    UART_Open(UART0, 9600);
    /* Configure UART1: 115200, 8-bit word, no parity bit, 1 stop bit. */
	printf("\n\nUART0:Fault Findder 485 Init\n");

    UART_Open(UART1, 115200);
	printf("\nUART1:Debug Terminal\n");


	UART0->MODEM &= ~UART_MODEM_RTSACTLV_Msk;
	UART0->MODEM |= UART_RTS_IS_HIGH_LEV_ACTIVE;
	UART_ENABLE_INT(UART0,(UART_INTEN_RDAIEN_Msk|UART_INTEN_RLSIEN_Msk));
	NVIC_EnableIRQ(UART0_IRQn);

}
void I2C0_Init(void)
{

    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);  /* iMON Comm */

    /* Get I2C0 Bus Clock */
	printf ("I2C clock iMON Comm  %d Hz\n",I2C_GetBusClockFreq(I2C0));


    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */



    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);


}

void I2C1_Init(void)
{

    /* Open I2C module and set bus clock */
	I2C_Open(I2C1,400000); /* Tempeture */

    /* Get I2C0 Bus Clock */

	printf ("I2C clock MPT100ON Comm  %d Hz\n",I2C_GetBusClockFreq(I2C1));




    /* Enable I2C interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);



}



uint8_t static count_i =0;
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{

	Delay(200);

	//printf("\nI2C  I2C_MasterRx Status 0x%x\n", u32Status);

    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C1, (g_u8MstDeviceAddr << 1));    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {

		 I2C_SET_DATA(I2C1, 0x00);
		 I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
		 g_u8MstDataLen = 1;
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C1);
        I2C_START(I2C1);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
	    count_i =0;

		I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA_SI);


    }	
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C1, ((g_u8MstDeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
       //I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
		
    }
	else if(u32Status == 0x50)                 /*50H, DATA received, ACK transmitted*/
	{


		 if(count_i < 2)
		 {

			 g_u8MstRxData[count_i] = (unsigned char) I2C_GET_DATA(I2C1);
			 I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
//			 printf("\n0x50 RX DATA : 0x%x\n",g_u8MstRxData[count_i]);
			 count_i++;
		 }
		 else if(count_i==2)
		 {
			 I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI_AA);
				 g_u8MstEndFlag = 1;
		 }	


	}
	else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
  #if 1
		 if(count_i != 2)
		 {

			 g_u8MstRxData[count_i] = (unsigned char) I2C_GET_DATA(I2C1);
			 I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
//			  printf("\n0x58 RX DATA : 0x%x\n",g_u8MstRxData[count_i]);
			 count_i++;
		 }
		 else
		 {
			 I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
				 g_u8MstEndFlag = 1;
		 }

#else
		 g_u8MstRxData = (unsigned char) I2C_GET_DATA(I2C1);
		 I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
		 g_u8MstEndFlag = 1;
	
	count_i++;

#endif

    }
    else
    {
        /* Error condition process */
        printf("[MasterRx] Status [0x%x] Unexpected abort!! Anykey to re-start\n", u32Status);
        if(u32Status == 0x38)                 /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)            /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)            /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)            /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        /*Setting MasterRx abort flag for re-start mechanism*/
        g_u8MstRxAbortFlag = 1;

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        u32TimeOutCnt =1000000 ;//I2C_TIMEOUT;
        while(I2C1->CTL & I2C_CTL_SI_Msk)
        {
            if(--u32TimeOutCnt == 0)
			{
				break;
            }
        }
    }

}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
	Delay(300);

	//printf("\nI2C  I2C_MasterTx Status 0x%x\n", u32Status);
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C1, g_u8MstDeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
      //  	printf("\nTX DATA : 0x%x\n",g_au8MstTxData[g_u8MstDataLen]);
        I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);

	//	Delay(10000);	
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        //I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI_AA);
        //I2C_STOP(I2C1);
        //I2C_START(I2C1);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != 3)
        {
          //   	printf("TX DATA[%x] : 0x%x\n",g_u8MstDataLen,g_au8MstTxData[g_u8MstDataLen]);
            I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
	    else
        {
//            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
	else if(u32Status == 0x30)   
	{
	          I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI_AA);
	}
    else
    {    /* Error condition process */
        printf("[MasterTx] Status [0x%x] Unexpected abort!! Anykey to re-start\n", u32Status);

        if(u32Status == 0x38)                   /* Master arbitration lost, stop I2C and clear SI */
        {
             I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA);
            //I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            //I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x10)              /* Master repeat start, clear SI */
        {
            I2C_SET_DATA(I2C1, (uint32_t)((g_u8MstDeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        /*Setting MasterTRx abort flag for re-start mechanism*/
        g_u8MstTxAbortFlag = 1;
    
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);

		u32TimeOutCnt = 3000000;//I2C_TIMEOUT 3000000 1sec
        while(I2C1->CTL & I2C_CTL_SI_Msk)
        {
            if(--u32TimeOutCnt == 0)
			{
				break;
            }
    	}

    }

}

int32_t I2C1_Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;
	uint8_t nResut =0;


    do
    {
        /* Enable I2C timeout */
        I2C_EnableTimeout(I2C1, 0);
        g_u8MstReStartFlag = 0;
        g_u8MstDeviceAddr = slvaddr;
        g_u8MstTimeoutFlag = 0;

        for(i = 0; i < 2; i++)
        {

	
            g_u8MstDataLen = 0;
            g_u8MstEndFlag = 0;
		

            /* I2C function to write data to slave */
            s_I2C1HandlerFn = (I2C_FUNC)I2C_MasterTx;

            /* I2C as master sends START signal */
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA);

            /* Wait I2C Tx Finish or Unexpected Abort */
            do
            {
                if(g_u8MstTimeoutFlag)
                {
                	set_i2sensor_status(i2c_sensor_fail);
                    printf(" MasterTx time out, any to reset IP\n");
                    SYS->IPRST1 |= SYS_IPRST1_I2C1RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C1_Init();
                    /* Set MasterTx abort flag */
                    g_u8MstTxAbortFlag = 1;
					nResut = 1;
                }
            } while(g_u8MstEndFlag == 0 && g_u8MstTxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstTxAbortFlag)
            {
                /* Clear MasterTx abort flag */
                g_u8MstTxAbortFlag = 0;
                /* Set Master re-start flag */
                g_u8MstReStartFlag = 1;
                break;
            }
			//Delay(1000);	
			Delay(100);	
            /* I2C function to read data from slave */
            s_I2C1HandlerFn = (I2C_FUNC)I2C_MasterRx;

            g_u8MstDataLen = 0;
            g_u8MstDeviceAddr = slvaddr;

            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA);

            /* Wait I2C Rx Finish or Unexpected Abort */
            do {
                if(g_u8MstTimeoutFlag)
                {
                	set_i2sensor_status(i2c_sensor_fail);
                    /* When I2C timeout, reset IP */
                    printf(" MasterRx time out, any to reset IP\n");
                    SYS->IPRST1 |= SYS_IPRST1_I2C1RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C1_Init();
                    /* Set MasterRx abort flag */
                    g_u8MstRxAbortFlag = 1;
					nResut = 1;
                }
            } while(g_u8MstEndFlag == 0 && g_u8MstRxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstRxAbortFlag )
            {
                /* Clear MasterRx abort flag */
                g_u8MstRxAbortFlag = 0;
                /* Set Master re-start flag */
                g_u8MstReStartFlag = 1;
                break;
            }
        }
    } while(g_u8MstReStartFlag); /*If unexpected abort happens, re-start the transmition*/

    
//	printf("I2C Byte Read Data [0x%02x] [0x%02x]and count 0x%x\n", g_u8MstRxData[0], g_u8MstRxData[1], count_i);

    
    return nResut;
}


/************************************************************************
 * DESCRIPTION:
 * INPUT      : none
 * RETURN     : none
 ************************************************************************/
void Delay(uint32_t delayCnt)
{
    while (delayCnt--) {
        __NOP();
        __NOP();
	    __NOP();
		__NOP();
		__NOP();
        __NOP();
	    __NOP();
		__NOP();
        __NOP();
	    __NOP();
		
		__NOP();		
        __NOP();
        __NOP();
	    __NOP();
		__NOP();
		__NOP();
        __NOP();
	    __NOP();
		__NOP();
		
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
uint16_t ModBus_CRC16 ( const unsigned char *buf, unsigned int len )
{
	static const uint16_t table[2] = { 0x0000, 0xA001 };
	uint16_t crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;
	unsigned int xor = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			xor = crc & 0x01;
			crc >>= 1;
			crc ^= table[xor];
		}
	}

	return crc;
	
}


uint8_t get_TemptureValue()
{
	static uint8_t k =0, printf_flag=0;
	static uint16_t temp13[8], temp23[8], adc_array[8][20]={0x0,};	
	static uint32_t raw_adc[8] = {0x0,};


	uint8_t i;

	/*
		
	ADS1115_CONFIG_REGISTER_MUX_DIFF_0_1	//(default)
	 ADS1115_CONFIG_REGISTER_MUX_DIFF_0_3	
	 ADS1115_CONFIG_REGISTER_MUX_DIFF_1_3	
	 ADS1115_CONFIG_REGISTER_MUX_DIFF_2_3	
	 ADS1115_CONFIG_REGISTER_MUX_SINGLE_0	
	 ADS1115_CONFIG_REGISTER_MUX_SINGLE_1	
	 ADS1115_CONFIG_REGISTER_MUX_SINGLE_2	
	 ADS1115_CONFIG_REGISTER_MUX_SINGLE_3	

	*/



   uint16_t setup_data = ADS1115_CONFIG_REGISTER_OS_NO_EFFECT  | /* 0x8000  ADS1115_CONFIG_REGISTER_OS_SINGLE */
			 ADS1115_CONFIG_REGISTER_PGA_2_048	  | 		 /* 0x0400 (default) ADS1115_CONFIG_REGISTER_PGA_2_048 */
			 ADS1115_CONFIG_REGISTER_MODE_CONTINUE  | 		 /* 0x0100 (default)  ADS1115_CONFIG_REGISTER_MODE_SINGLE */  
			 ADS1115_CONFIG_REGISTER_DR_128_SPS   | 		 /* 0x0080 (default) ADS1115_CONFIG_REGISTER_DR_128_SPS */  
			 ADS1115_CONFIG_REGISTER_COMP_MODE_TRADITIONAL_COMPARATOR	|/* 0x0000 (default) ADS1115_CONFIG_REGISTER_COMP_MODE_TRADITIONAL_COMPARATOR */ 	 
			 ADS1115_CONFIG_REGISTER_COMP_POL_ACTIVE_LOW |/* 0x0000 (default) ADS1115_CONFIG_REGISTER_COMP_POL_ACTIVE_LOW */ 
			 ADS1115_CONFIG_REGISTER_COMP_LAT_NONE		 |/* 0x0000 (default) ADS1115_CONFIG_REGISTER_COMP_LAT_NONE */ 
			 ADS1115_CONFIG_REGISTER_COMP_QUE_DISABLE;	  /* 0x0003 (default) ADS1115_CONFIG_REGISTER_COMP_QUE_DISABLE */ 


	
	if(k >= 15)
	{
		k = 0;
	}

	/* ############## ADS1115_CONFIG_REGISTER_MUX_DIFF_1_3	*/
	
	g_au8MstTxData[0] = 0x01;
	g_au8MstTxData[2] = setup_data & 0x00FF;





	if(get_Sensor1OnOff() == 1 )
	{

		temp13[SENSOR_REDUCER_IDX1]=0;
		temp23[SENSOR_REDUCER_IDX1]=0;	
		raw_adc[SENSOR_REDUCER_IDX1]=0;

		PA15= (SENSOR_REDUCER_IDX1 & 0x04 ) >> 2;/* S2 */
		PA14= (SENSOR_REDUCER_IDX1  & 0x02) >> 1;/* S1 */
		PA13= SENSOR_REDUCER_IDX1	& 0x1; /* S0 */
	
		PA12 = 0;
		Delay(50000);			

		PA->DOUT = (PA->DOUT|BIT1)&(~(1<<1));
	
			g_au8MstTxData[1] = ((setup_data | ADS1115_CONFIG_REGISTER_MUX_DIFF_1_3)  >> 8) & 0xFF ; /* Reset 0x06 */

			if(I2C1_Read_Write_SLAVE(ADS1115_ADDRESS) == 0)
			{
			 	//printf("\n######################RX	RX	RX	 DATA : [0x%02x][0x%02x]\n",g_u8MstRxData[0],g_u8MstRxData[1]);
//				printf("Diff 13  %02x:%02x\n",g_u8MstRxData[0], g_u8MstRxData[1] );
				
			 	temp13[SENSOR_REDUCER_IDX1] = 0;
				temp13[SENSOR_REDUCER_IDX1] = ((g_u8MstRxData[0] << 8 ) & 0xFF00)  + g_u8MstRxData[1] ;

			}
			else
			{
				//printf("TEMP At %d:%d  [%d Real Data[%d]\n",iner_temp_cnt, k, 	temp[iner_temp_cnt], adc_array[iner_temp_cnt][k]);
			}



			Delay(50000);

			g_au8MstTxData[1] = ((setup_data | ADS1115_CONFIG_REGISTER_MUX_DIFF_2_3)  >> 8) & 0xFF ; /* Reset 0x06 */

			if(I2C1_Read_Write_SLAVE(ADS1115_ADDRESS) == 0)
			{
//				printf("Diff 23  %02x:%02x\n",g_u8MstRxData[0], g_u8MstRxData[1] );
				
				temp23[SENSOR_REDUCER_IDX1] = 0;
				temp23[SENSOR_REDUCER_IDX1] = ((g_u8MstRxData[0] << 8 ) & 0xFF00)  + g_u8MstRxData[1] ;
			}
			else
			{
				//printf("TEMP At %d:%d  [%d Real Data[%d]\n",iner_temp_cnt, k, 	temp[iner_temp_cnt], adc_array[iner_temp_cnt][k]);
			}



			if(get_i2sensor_status() == i2c_sensor_normal)
			{
				adc_array[SENSOR_REDUCER_IDX1][k]= (((2*temp23[SENSOR_REDUCER_IDX1]) - temp13[SENSOR_REDUCER_IDX1] - temp23[SENSOR_REDUCER_IDX1])* 10000) / temp13[SENSOR_REDUCER_IDX1] ; 
				printf("SENSOR_REDUCER_IDX1 ADC %d. [%d]",SENSOR_REDUCER_IDX1+1, adc_array[SENSOR_REDUCER_IDX1][k]);
			}
			else if(get_i2sensor_status() == i2c_sensor_fail)
			{
				set_i2sensor_status(i2c_sensor_normal);
			}


			for( i=0 ; i < 15 ;i++)
			{
				 raw_adc[SENSOR_REDUCER_IDX1] = raw_adc[SENSOR_REDUCER_IDX1] + adc_array[SENSOR_REDUCER_IDX1][i];
				// printf("DATA %d [%d]\n",i, adc_array[iner_temp_cnt][i]);
			}



			 raw_adc[SENSOR_REDUCER_IDX1] = raw_adc[SENSOR_REDUCER_IDX1] / 15 ;
			if(k== (15 - 1) )
			 {
				 printf_flag = 1;
			 }	 




			if(printf_flag==1)
			 {
				printf(" [Avg: %d]",  raw_adc[SENSOR_REDUCER_IDX1]);
				


				 if(raw_adc[SENSOR_REDUCER_IDX1] > pt100_table[200])
				 {
					raw_adc[SENSOR_REDUCER_IDX1] = pt100_table[200];
				 }
				 else if(raw_adc[SENSOR_REDUCER_IDX1] < pt100_table[0])
				 {
					raw_adc[SENSOR_REDUCER_IDX1] = pt100_table[0];

				 }


				 for(i=0 ; i < 201 ; i++)
				 {
				 	if(raw_adc[SENSOR_REDUCER_IDX1] < pt100_table[i])
				 	{
				 		g_tempture_value[SENSOR_REDUCER_IDX1] =i;
						break;
				 	}
				 }

			
			     printf(" Real [%d] C\n",  g_tempture_value[SENSOR_REDUCER_IDX1]);
			 }
			

			
	}
	else
	{
		temp13[SENSOR_REDUCER_IDX1] = 0x0;
		temp23[SENSOR_REDUCER_IDX1] = 0x0;
		adc_array[SENSOR_REDUCER_IDX1][k]=0x0;
		g_tempture_value[SENSOR_REDUCER_IDX1]=0xff;
		//printf(" SENSOR_REDUCER_IDX1 Disabled\n");
	}
	PA12 = 1;


	
		if(get_Sensor2OnOff() == 1 )
		{
	
			temp13[SENSOR_LINING1_IIDX2]=0;
			temp23[SENSOR_LINING1_IIDX2]=0;	
			raw_adc[SENSOR_LINING1_IIDX2]=0;
	
			PA15= (SENSOR_LINING1_IIDX2 & 0x04 ) >> 2;/* S2 */
			PA14= (SENSOR_LINING1_IIDX2  & 0x02) >> 1;/* S1 */
			PA13= SENSOR_LINING1_IIDX2 & 0x1; /* S0 */
		
			PA12 = 0;
			Delay(50000);			
	
			PA->DOUT = (PA->DOUT|BIT1)&(~(1<<1));
		
				g_au8MstTxData[1] = ((setup_data | ADS1115_CONFIG_REGISTER_MUX_DIFF_1_3)  >> 8) & 0xFF ; /* Reset 0x06 */
	
				if(I2C1_Read_Write_SLAVE(ADS1115_ADDRESS) == 0)
				{
					//printf("\n######################RX	RX	RX	 DATA : [0x%02x][0x%02x]\n",g_u8MstRxData[0],g_u8MstRxData[1]);
	//				printf("Diff 13  %02x:%02x\n",g_u8MstRxData[0], g_u8MstRxData[1] );
					
					temp13[SENSOR_LINING1_IIDX2] = 0;
					temp13[SENSOR_LINING1_IIDX2] = ((g_u8MstRxData[0] << 8 ) & 0xFF00)  + g_u8MstRxData[1] ;
	
				}
				else
				{
					//printf("TEMP At %d:%d  [%d Real Data[%d]\n",iner_temp_cnt, k, 	temp[iner_temp_cnt], adc_array[iner_temp_cnt][k]);
				}
	
	
	
				Delay(50000);
	
				g_au8MstTxData[1] = ((setup_data | ADS1115_CONFIG_REGISTER_MUX_DIFF_2_3)  >> 8) & 0xFF ; /* Reset 0x06 */
	
				if(I2C1_Read_Write_SLAVE(ADS1115_ADDRESS) == 0)
				{
	//				printf("Diff 23  %02x:%02x\n",g_u8MstRxData[0], g_u8MstRxData[1] );
					
					temp23[SENSOR_LINING1_IIDX2] = 0;
					temp23[SENSOR_LINING1_IIDX2] = ((g_u8MstRxData[0] << 8 ) & 0xFF00)  + g_u8MstRxData[1] ;
				}
				else
				{
					//printf("TEMP At %d:%d  [%d Real Data[%d]\n",iner_temp_cnt, k, 	temp[iner_temp_cnt], adc_array[iner_temp_cnt][k]);
				}
	
	
	
				if(get_i2sensor_status() == i2c_sensor_normal)
				{
					adc_array[SENSOR_LINING1_IIDX2][k]= (((2*temp23[SENSOR_LINING1_IIDX2]) - temp13[SENSOR_LINING1_IIDX2] - temp23[SENSOR_LINING1_IIDX2])* 10000) / temp13[SENSOR_LINING1_IIDX2] ; 
					printf("SENSOR_LINING1_IDX2 ADC %d. [%d]",SENSOR_LINING1_IIDX2+1, adc_array[SENSOR_LINING1_IIDX2][k]);
				}
				else if(get_i2sensor_status() == i2c_sensor_fail)
				{
					set_i2sensor_status(i2c_sensor_normal);
				}
	
	
				for( i=0 ; i < 15 ;i++)
				{
					 raw_adc[SENSOR_LINING1_IIDX2] = raw_adc[SENSOR_LINING1_IIDX2] + adc_array[SENSOR_LINING1_IIDX2][i];
					// printf("DATA %d [%d]\n",i, adc_array[iner_temp_cnt][i]);
				}
	
	
	
				 raw_adc[SENSOR_LINING1_IIDX2] = raw_adc[SENSOR_LINING1_IIDX2] / 15 ;
				if(k== (15 - 1) )
				 {
					 printf_flag = 1;
				 }	 
	
	
	
	
				if(printf_flag==1)
				 {
					printf(" [Avg: %d]",  raw_adc[SENSOR_LINING1_IIDX2]);
					
	
	
					 if(raw_adc[SENSOR_LINING1_IIDX2] > pt100_table[200])
					 {
						raw_adc[SENSOR_LINING1_IIDX2] = pt100_table[200];
					 }
					 else if(raw_adc[SENSOR_LINING1_IIDX2] < pt100_table[0])
					 {
						raw_adc[SENSOR_LINING1_IIDX2] = pt100_table[0];
	
					 }
	
	
					 for(i=0 ; i < 201 ; i++)
					 {
						if(raw_adc[SENSOR_LINING1_IIDX2] < pt100_table[i])
						{
							g_tempture_value[SENSOR_LINING1_IIDX2] =i;
							break;
						}
					 }
	
				
					 printf(" Real [%d] C\n",  g_tempture_value[SENSOR_LINING1_IIDX2]);
				 }
				
	
				
		}
		else
		{
			temp13[SENSOR_LINING1_IIDX2] = 0x0;
			temp23[SENSOR_LINING1_IIDX2] = 0x0;
			adc_array[SENSOR_LINING1_IIDX2][k]=0x0;
			g_tempture_value[SENSOR_LINING1_IIDX2]=0xff;
			//printf(" SENSOR_REDUCER_IDX1 Disabled\n");

		}
		PA12 = 1;
	

		if(get_Sensor3OnOff() == 1 )
		{
	
			temp13[SENSOR_LINING2_IDX3]=0;
			temp23[SENSOR_LINING2_IDX3]=0;	
			raw_adc[SENSOR_LINING2_IDX3]=0;
	
			PA15= (SENSOR_LINING2_IDX3 & 0x04 ) >> 2;/* S2 */
			PA14= (SENSOR_LINING2_IDX3  & 0x02) >> 1;/* S1 */
			PA13= SENSOR_LINING2_IDX3 & 0x1; /* S0 */
		
			PA12 = 0;
			Delay(50000);			
	
			PA->DOUT = (PA->DOUT|BIT1)&(~(1<<1));
		
				g_au8MstTxData[1] = ((setup_data | ADS1115_CONFIG_REGISTER_MUX_DIFF_1_3)  >> 8) & 0xFF ; /* Reset 0x06 */
	
				if(I2C1_Read_Write_SLAVE(ADS1115_ADDRESS) == 0)
				{
					//printf("\n######################RX	RX	RX	 DATA : [0x%02x][0x%02x]\n",g_u8MstRxData[0],g_u8MstRxData[1]);
	//				printf("Diff 13  %02x:%02x\n",g_u8MstRxData[0], g_u8MstRxData[1] );
					
					temp13[SENSOR_LINING2_IDX3] = 0;
					temp13[SENSOR_LINING2_IDX3] = ((g_u8MstRxData[0] << 8 ) & 0xFF00)  + g_u8MstRxData[1] ;
	
				}
				else
				{
					//printf("TEMP At %d:%d  [%d Real Data[%d]\n",iner_temp_cnt, k, 	temp[iner_temp_cnt], adc_array[iner_temp_cnt][k]);
				}
	
	
	
				Delay(50000);
	
				g_au8MstTxData[1] = ((setup_data | ADS1115_CONFIG_REGISTER_MUX_DIFF_2_3)  >> 8) & 0xFF ; /* Reset 0x06 */
	
				if(I2C1_Read_Write_SLAVE(ADS1115_ADDRESS) == 0)
				{
	//				printf("Diff 23  %02x:%02x\n",g_u8MstRxData[0], g_u8MstRxData[1] );
					
					temp23[SENSOR_LINING2_IDX3] = 0;
					temp23[SENSOR_LINING2_IDX3] = ((g_u8MstRxData[0] << 8 ) & 0xFF00)  + g_u8MstRxData[1] ;
				}
				else
				{
					//printf("TEMP At %d:%d  [%d Real Data[%d]\n",iner_temp_cnt, k, 	temp[iner_temp_cnt], adc_array[iner_temp_cnt][k]);
				}
	
	
	
				if(get_i2sensor_status() == i2c_sensor_normal)
				{
					adc_array[SENSOR_LINING2_IDX3][k]= (((2*temp23[SENSOR_LINING2_IDX3]) - temp13[SENSOR_LINING2_IDX3] - temp23[SENSOR_LINING2_IDX3])* 10000) / temp13[SENSOR_LINING2_IDX3] ; 
					printf("SENSOR_LINING2_IDX3 ADC %d. [%d]",SENSOR_LINING2_IDX3+1, adc_array[SENSOR_LINING2_IDX3][k]);
				}
				else if(get_i2sensor_status() == i2c_sensor_fail)
				{
					set_i2sensor_status(i2c_sensor_normal);
				}
	
	
				for( i=0 ; i < 15 ;i++)
				{
					 raw_adc[SENSOR_LINING2_IDX3] = raw_adc[SENSOR_LINING2_IDX3] + adc_array[SENSOR_LINING2_IDX3][i];
					// printf("DATA %d [%d]\n",i, adc_array[iner_temp_cnt][i]);
				}
	
	
	
				 raw_adc[SENSOR_LINING2_IDX3] = raw_adc[SENSOR_LINING2_IDX3] / 15 ;
				if(k== (15 - 1) )
				 {
					 printf_flag = 1;
				 }	 
	
	
	
	
				if(printf_flag==1)
				 {
					printf(" [Avg: %d]",  raw_adc[SENSOR_LINING2_IDX3]);
					
	
	
					 if(raw_adc[SENSOR_LINING2_IDX3] > pt100_table[200])
					 {
						raw_adc[SENSOR_LINING2_IDX3] = pt100_table[200];
					 }
					 else if(raw_adc[SENSOR_LINING2_IDX3] < pt100_table[0])
					 {
						raw_adc[SENSOR_LINING2_IDX3] = pt100_table[0];
	
					 }
	
	
					 for(i=0 ; i < 201 ; i++)
					 {
						if(raw_adc[SENSOR_LINING2_IDX3] < pt100_table[i])
						{
							g_tempture_value[SENSOR_LINING2_IDX3] =i;
							break;
						}
					 }
	
				
					 printf(" Real [%d] C\n",  g_tempture_value[SENSOR_LINING2_IDX3]);
				 }
				
	
				
		}
		else
		{
			temp13[SENSOR_LINING2_IDX3] = 0x0;
			temp23[SENSOR_LINING2_IDX3] = 0x0;
			adc_array[SENSOR_LINING2_IDX3][k]=0x0;
			g_tempture_value[SENSOR_LINING2_IDX3]=0xff;
			//printf(" SENSOR_REDUCER_IDX1 Disabled\n");
		}
		PA12 = 1;
	

		if(get_Sensor4OnOff() == 1 )
		{
	
			temp13[SENSOR_BRKCOIL_IDX4]=0;
			temp23[SENSOR_BRKCOIL_IDX4]=0;	
			raw_adc[SENSOR_BRKCOIL_IDX4]=0;
	
			PA15= (SENSOR_BRKCOIL_IDX4 & 0x04 ) >> 2;/* S2 */
			PA14= (SENSOR_BRKCOIL_IDX4  & 0x02) >> 1;/* S1 */
			PA13= SENSOR_BRKCOIL_IDX4 & 0x1; /* S0 */
		
			PA12 = 0;
			Delay(50000);			
	
			PA->DOUT = (PA->DOUT|BIT1)&(~(1<<1));
		
				g_au8MstTxData[1] = ((setup_data | ADS1115_CONFIG_REGISTER_MUX_DIFF_1_3)  >> 8) & 0xFF ; /* Reset 0x06 */
	
				if(I2C1_Read_Write_SLAVE(ADS1115_ADDRESS) == 0)
				{
					//printf("\n######################RX	RX	RX	 DATA : [0x%02x][0x%02x]\n",g_u8MstRxData[0],g_u8MstRxData[1]);
	//				printf("Diff 13  %02x:%02x\n",g_u8MstRxData[0], g_u8MstRxData[1] );
					
					temp13[SENSOR_BRKCOIL_IDX4] = 0;
					temp13[SENSOR_BRKCOIL_IDX4] = ((g_u8MstRxData[0] << 8 ) & 0xFF00)  + g_u8MstRxData[1] ;
	
				}
				else
				{
					//printf("TEMP At %d:%d  [%d Real Data[%d]\n",iner_temp_cnt, k, 	temp[iner_temp_cnt], adc_array[iner_temp_cnt][k]);
				}
	
	
	
				Delay(50000);
	
				g_au8MstTxData[1] = ((setup_data | ADS1115_CONFIG_REGISTER_MUX_DIFF_2_3)  >> 8) & 0xFF ; /* Reset 0x06 */
	
				if(I2C1_Read_Write_SLAVE(ADS1115_ADDRESS) == 0)
				{
	//				printf("Diff 23  %02x:%02x\n",g_u8MstRxData[0], g_u8MstRxData[1] );
					
					temp23[SENSOR_BRKCOIL_IDX4] = 0;
					temp23[SENSOR_BRKCOIL_IDX4] = ((g_u8MstRxData[0] << 8 ) & 0xFF00)  + g_u8MstRxData[1] ;
				}
				else
				{
					//printf("TEMP At %d:%d  [%d Real Data[%d]\n",iner_temp_cnt, k, 	temp[iner_temp_cnt], adc_array[iner_temp_cnt][k]);
				}
	
	
	
				if(get_i2sensor_status() == i2c_sensor_normal)
				{
					adc_array[SENSOR_BRKCOIL_IDX4][k]= (((2*temp23[SENSOR_BRKCOIL_IDX4]) - temp13[SENSOR_BRKCOIL_IDX4] - temp23[SENSOR_BRKCOIL_IDX4])* 10000) / temp13[SENSOR_BRKCOIL_IDX4] ; 
					printf("SENSOR_BRKCOIL_IDX4 ADC %d. [%d]",SENSOR_BRKCOIL_IDX4+1, adc_array[SENSOR_BRKCOIL_IDX4][k]);
				}
				else if(get_i2sensor_status() == i2c_sensor_fail)
				{
					set_i2sensor_status(i2c_sensor_normal);
				}
	
	
				for( i=0 ; i < 15 ;i++)
				{
					 raw_adc[SENSOR_BRKCOIL_IDX4] = raw_adc[SENSOR_BRKCOIL_IDX4] + adc_array[SENSOR_BRKCOIL_IDX4][i];
					// printf("DATA %d [%d]\n",i, adc_array[iner_temp_cnt][i]);
				}
	
	
	
				 raw_adc[SENSOR_BRKCOIL_IDX4] = raw_adc[SENSOR_BRKCOIL_IDX4] / 15 ;
				if(k== (15 - 1) )
				 {
					 printf_flag = 1;
				 }	 
	
	
	
	
				if(printf_flag==1)
				 {
					printf(" [Avg: %d]",  raw_adc[SENSOR_BRKCOIL_IDX4]);
					
	
	
					 if(raw_adc[SENSOR_BRKCOIL_IDX4] > pt100_table[200])
					 {
						raw_adc[SENSOR_BRKCOIL_IDX4] = pt100_table[200];
					 }
					 else if(raw_adc[SENSOR_BRKCOIL_IDX4] < pt100_table[0])
					 {
						raw_adc[SENSOR_BRKCOIL_IDX4] = pt100_table[0];
	
					 }
	
	
					 for(i=0 ; i < 201 ; i++)
					 {
						if(raw_adc[SENSOR_BRKCOIL_IDX4] < pt100_table[i])
						{
							g_tempture_value[SENSOR_BRKCOIL_IDX4] =i;
							break;
						}
					 }
	
				
					 printf(" Real [%d] C\n",  g_tempture_value[SENSOR_BRKCOIL_IDX4]);
				 }
				
	
				
		}
		else
		{
			temp13[SENSOR_BRKCOIL_IDX4] = 0x0;
			temp23[SENSOR_BRKCOIL_IDX4] = 0x0;
			adc_array[SENSOR_BRKCOIL_IDX4][k]=0x0;
			g_tempture_value[SENSOR_BRKCOIL_IDX4]=0xff;
			//printf(" SENSOR_REDUCER_IDX1 Disabled\n");
		}
		PA12 = 1;
	

	printf("\n");

	k++;





	return 0;

}



uint8_t get_SensorSelect()
{
	uint8_t pin_array[3]={0x0,},get_pin_value=0;


	pin_array[2]= 0x04  & ((0x1  & ~PB0) << 2 );
	pin_array[1]= 0x02  & ((0x1  & ~PB1) << 1 );
	pin_array[0]= 0x01  & (0x1  & ~PB2);
	
	get_pin_value = pin_array[2] | \
					pin_array[1] | \
					pin_array[0];



	return get_pin_value;

}

uint8_t get_SensorOnOff()
{
	uint8_t pin_array=0x0;

	pin_array = 0x01  & (0x1  & ~PB3) ;

	
	return pin_array;

}



uint8_t get_Sensor4OnOff()
{
	uint8_t pin_array=0x0;

	pin_array = 0x01  & (0x1  & ~PB0) ;

	
	return pin_array;

}



uint8_t get_Sensor3OnOff()
{
	uint8_t pin_array=0x0;

	pin_array = 0x01  & (0x1  & ~PB1) ;

	
	return pin_array;

}


uint8_t get_Sensor2OnOff()
{
	uint8_t pin_array=0x0;

	pin_array = 0x01  & (0x1  & ~PB2) ;

	
	return pin_array;

}


uint8_t get_Sensor1OnOff()
{
	uint8_t pin_array=0x0;

	pin_array = 0x01  & (0x1  & ~PB3) ;

	
	return pin_array;

}


uint8_t get_PinOilLevelValue()
{
	uint8_t pin_array=0x0;

	pin_array = 0x01  & (0x1  & ~PB4) ;

	
	return pin_array;


}


uint8_t get_PinValue()
{
	uint8_t pin_array[3]={0x0,},get_pin_value=0;


	pin_array[2]= 0x04  & ((0x1  & ~PB5) << 2 );
	pin_array[1]= 0x02  & ((0x1  & ~PB6) << 1 );
	pin_array[0]= 0x01  & (0x1  & ~PB7 );

	get_pin_value = pin_array[2] | \
					pin_array[1] | \
					pin_array[0];


	return get_pin_value;

}
/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
	uint8_t u485Recv_Cnt=0, crc_finderData[2];
    uint32_t i, u32TimeOutCnt;
	int local_count=0;
	uint16_t crc16_result = 0x0;

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

    UART_Init();
	    /* Init SPI */
    SPI_Init();

	I2C0_Init();
	I2C1_Init();


	Timer_Init();




#if 0 /* NuConsole without UART */ 
    NuConsole_Init(); 
#endif


 #if 1

	PC4 = 1;  // DOUT_EN Active Low
	PC3 = 1; 	// DOUT_EX1
	PC2 = 0; 	// DOUT_EX2
	PC1 = 1;  // DOUT_EX3
	PC0 = 0;  // DOUT_EX4


 #endif
	set_CRC16_faultfinder();

	printf("\nDIP SWITCH Value [0x%x]\n\n",get_PinValue());



    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    /* I2C function to Slave receive/transmit data */
    s_I2C0HandlerFn = I2C_SlaveTRx;

    printf("\n");
    printf("I2C  Mode is Running.\n");

	  


  	TEMP_save =	 TEMP_Cnt / 1000000 ;


	Delay(1000000); //1000000 1 Sec
	Delay(1000000); //1000000 1 Sec
	Delay(1000000); //1000000 1 Sec
	Delay(1000000); //1000000 1 Sec
	Delay(1000000); //1000000 1 Sec




    printf("\n\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("|                   iMON E/S Communication Code                      |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");
	I2C_Transmit_clean();

    while(1)
    {
    	if(local_count == 1)
		{	
			RS485_SendDataByte(g_fault_finderData, 8);
			local_count=0;
			
			if(u485Recv_Cnt > 3)
			{
				set_485comm_status(comm_485_error);	
				I2C_Transmit_clean();
			}
			else
			{
				u485Recv_Cnt++;
			}
    	}
    	local_count++;
    	
    	
    	//if(g_485_flags==1 && g_u32_485RxDataCount == 17)
    	if(g_485_flags==1 && g_u32_485RxDataCount > 1)
		{

			printf("\nFault Finder Data Cnt %d\n",g_u32_485RxDataCount);
#if 0

			for(i=0;i<g_u32_485RxDataCount;i++)
			{
				 printf("[%d:0x%02x] ", i,g_fault_RecvData[i]);
	
			}
			printf("\n ");
#endif
		
			crc16_result = ModBus_CRC16(g_fault_RecvData,15);
			
			crc_finderData[1] = (crc16_result >> 8) & 0xff;
			crc_finderData[0] = crc16_result & 0xff;  

			

			if(crc_finderData[0] == g_fault_RecvData[15] && crc_finderData[1] == g_fault_RecvData[16] )
			{
				I2C_Transmit_made();
			}
			else
			{
				printf("Result CRC16 Wrong [0x%02x:0x%02x] =! Received [0x%02x:0x%02x]\n", crc_finderData[0],crc_finderData[1], \
																				g_fault_RecvData[15],g_fault_RecvData[16]);
				I2C_Transmit_clean();
			}
			g_485_flags = 0 ;
			g_u32_485RxDataCount=0;
			u485Recv_Cnt= 0;
			set_485comm_status(comm_485_done);
			//I2C_Transmit_made_test();
		   	
		}

        /* Handle Slave timeout condition */
        if(g_u8SlvTimeoutFlag)
        {
            printf(" SlaveTRx time out, any to reset IP\n");
            SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
            SYS->IPRST1 = 0;
            I2C0_Init();
            g_u8SlvTimeoutFlag = 0;
            g_u8SlvTRxAbortFlag = 1;
        }
        /* When I2C abort, clear SI to enter non-addressed SLV mode*/
        if(g_u8SlvTRxAbortFlag)
        {
            g_u8SlvTRxAbortFlag = 0;
            u32TimeOutCnt = 10000000;//I2C_TIMEOUT;
            while(I2C0->CTL & I2C_CTL_SI_Msk)
            {
                if(--u32TimeOutCnt == 0)
				{
					break;
                }
            }
            printf("I2C Slave re-start. status[0x%x]\n", I2C0->STATUS);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }



 #if 0
 		
 		if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI2) == 0)
		{
			printf("SPI RECEIVE [0x%2x]\n ",SPI_READ_RX(SPI2));
 		}

#endif

#if 0
		if(PD15==1)
		{
				set_oillevel_status (oil_level_normal);
		}
		else
		{
				if(local_count == 25)
				{
					printf("OIL Level DIN4 Warning\n");// DIN4
				}
				set_oillevel_status(oil_level_warning);

		}

#endif

#if 0
		if(PD14==1)
		{
				set_oillevel_status (oil_level_normal);
		}
		else
		{
				if(local_count == 25)
				{
					printf("OIL Level DIN3 Warning\n");// DIN3
				}
				set_oillevel_status(oil_level_warning);

		}
#endif


#if 0
		if(PD13==1)
		{
				set_oillevel_status (oil_level_normal);
		}
		else
		{
				if(local_count == 25)
				{
					printf("OIL Level DIN2 Warning\n");// DIN2
				}
				set_oillevel_status(oil_level_warning);

		}
#endif
		
		if(PD12==1)
		{
				set_oillevel_status(oil_level_normal);
		}
		else
		{		
				if(local_count == 3)
				{
					printf("OIL Level DIN1 Warning\n");// DIN1
				}
				set_oillevel_status(oil_level_warning);
		}

		


		

#if 1
		Delay(200000);

#else
		Delay(50000);
#endif
		

		
		PA->DOUT = (PA->DOUT|BIT1)&(~(0<<1));

		   



		if(TEMP_save != TEMP_Cnt / 1000000)
		{

			TEMP_save = TEMP_Cnt / 1000000;
			get_TemptureValue();

		}



 
   }
   
	

}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

