


#define ADS1115_ADDRESS 0x48

#define ADS1115_CONVERSION_REGISTER_ADDRESS	0x00
#define ADS1115_CONFIG_REGISTER_ADDRESS		0x01

#define ADS1115_CONFIG_REGISTER_OS_NO_EFFECT	0x0000
#define ADS1115_CONFIG_REGISTER_OS_SINGLE	0x8000

#define ADS1115_CONFIG_REGISTER_MUX_DIFF_0_1	0x0000//(default)
#define ADS1115_CONFIG_REGISTER_MUX_DIFF_0_3	0x1000
#define ADS1115_CONFIG_REGISTER_MUX_DIFF_1_3	0x2000
#define ADS1115_CONFIG_REGISTER_MUX_DIFF_2_3	0x3000
#define ADS1115_CONFIG_REGISTER_MUX_SINGLE_0	0x4000
#define ADS1115_CONFIG_REGISTER_MUX_SINGLE_1	0x5000
#define ADS1115_CONFIG_REGISTER_MUX_SINGLE_2	0x6000
#define ADS1115_CONFIG_REGISTER_MUX_SINGLE_3	0x7000

#define ADS1115_CONFIG_REGISTER_PGA_6_144	0x0000
#define ADS1115_CONFIG_REGISTER_PGA_4_096	0x0200
#define ADS1115_CONFIG_REGISTER_PGA_2_048	0x0400//(default)
#define ADS1115_CONFIG_REGISTER_PGA_1_024	0x0600
#define ADS1115_CONFIG_REGISTER_PGA_0_512	0x0800
#define ADS1115_CONFIG_REGISTER_PGA_0_256	0x0A00

#define ADS1115_CONFIG_REGISTER_MODE_CONTINUE	0x0000
#define ADS1115_CONFIG_REGISTER_MODE_SINGLE	0x0100//(default)

#define ADS1115_CONFIG_REGISTER_DR_8_SPS	0x0000
#define ADS1115_CONFIG_REGISTER_DR_16_SPS	0x0020
#define ADS1115_CONFIG_REGISTER_DR_32_SPS	0x0040
#define	ADS1115_CONFIG_REGISTER_DR_64_SPS	0x0060
#define ADS1115_CONFIG_REGISTER_DR_128_SPS	0x0080//(default)
#define ADS1115_CONFIG_REGISTER_DR_800_SPS  0x00E0



#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000

enum comm_485_status {
    comm_485_ready = 0 ,
    comm_485_done = 1,
    comm_485_error = 2
};

enum oil_level_status {
    oil_level_ready = 2 ,
    oil_level_normal = 1,
    oil_level_warning = 0
};


enum i2c_sensor_status {
    i2c_sensor_ready = 2 ,
    i2c_sensor_normal = 1,
    i2c_sensor_fail = 0
};


enum sensor_idx {
    SENSOR_REDUCER_IDX1 = 0 ,
    SENSOR_LINING1_IIDX2 = 1,
    SENSOR_LINING2_IDX3 = 2,
    SENSOR_BRKCOIL_IDX4 = 3
};

typedef void (*I2C_FUNC)(uint32_t u32Status);

#define EXTEN_BD_TYPE 0x01

float PID(float in);

void set_i2sensor_status(uint8_t set_status);
uint8_t get_i2sensor_status();
void set_oillevel_status(uint8_t set_status);
uint8_t get_oillevel_status();
void set_485comm_status(uint8_t set_status);
uint8_t get_485comm_status();

void I2C0_IRQHandler(void);
void TMR0_IRQHandler(void);
void TMR1_IRQHandler(void);
void UART0_IRQHandler(void);
void SPI2_IRQHandler(void);
int I2C_Reeceived_valid();
void set_CRC16_faultfinder();
int I2C_Transmit_made_test();
void get_TemptureWarningCheck();
int I2C_Transmit_clean();
int I2C_Transmit_made();

void I2C_SlaveTRx(uint32_t u32Status);
void I2C1_IRQHandler(void);
void SYS_Init(void);
void Timer_Init(void);
void SPI_Init(void);
void UART_Init(void);
void I2C0_Init(void);
void I2C1_Init(void);
void I2C_MasterRx(uint32_t u32Status);
void I2C_MasterTx(uint32_t u32Status);
int32_t I2C1_Read_Write_SLAVE(uint8_t slvaddr);
void Delay(uint32_t delayCnt);
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_ReadDataByte(uint8_t *puRTxBuf, uint32_t u32ReadBytes);
uint16_t ModBus_CRC16 ( const unsigned char *buf, unsigned int len );
uint8_t get_TemptureValue();
uint8_t get_SensorSelect();
uint8_t get_SensorOnOff();

uint8_t get_Sensor4OnOff();
uint8_t get_Sensor3OnOff();
uint8_t get_Sensor2OnOff();
uint8_t get_Sensor1OnOff();
uint8_t get_PinOilLevelValue();
uint8_t get_PinValue();


















































































































































































































































































































































































#define ADS1115_CONFIG_REGISTER_DR_250_SPS	0x00A0
#define ADS1115_CONFIG_REGISTER_DR_475_SPS	0x00C0
#define ADS1115_CONFIG_REGISTER_DR_800_SPS	0x00E0

#define ADS1115_CONFIG_REGISTER_COMP_MODE_TRADITIONAL_COMPARATOR	0x0000//(default)
#define ADS1115_CONFIG_REGISTER_CONP_MODE_WINDOW_COMPARATOR		0x0010

#define ADS1115_CONFIG_REGISTER_COMP_POL_ACTIVE_LOW			0x0000//(default)
#define ADS1115_CONFIG_REGISTER_COMP_POL_ACTIVE_HIGH			0x0080

#define ADS1115_CONFIG_REGISTER_COMP_LAT_NONE				0x0000//(default)
#define ADS1115_CONFIG_REGISTER_COMP_LAT				0x0004

#define ADS1115_CONFIG_REGISTER_COMP_QUE_1CONV				0x0000
#define ADS1115_CONFIG_REGISTER_COMP_QUE_2CONV				0x0001
#define ADS1115_CONFIG_REGISTER_COMP_QUE_4CONV				0x0002
#define ADS1115_CONFIG_REGISTER_COMP_QUE_DISABLE			0x0003 //(default)
/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);
void UART_Init(void);
void I2C0_Init(void);

void I2C1_Init(void);


void Timer_Init(void);
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);
void SPI2_IRQHandler(void);
void UART0_IRQHandler(void);
void I2C_SlaveTRx(uint32_t u32Status);
int32_t I2C1_Read_Write_SLAVE(uint8_t slvaddr);


void TMR1_IRQHandler(void);
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
void Delay(uint32_t delayCnt);
uint16_t ModBus_CRC16 ( const unsigned char *buf, unsigned int len );

uint8_t get_PinValue();
uint8_t get_SensorSelect();

uint8_t get_SensorOnOff();

