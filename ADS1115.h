


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






const uint16_t pt100_table[201] = {
	10000, 	10039,	10078,	10117,	10156,	10195,	10234,	10273,	10312,	10351,   //   0 -   9 
	10390,	10429,	10468,  10507,	10546,	10585,	10624,	10663,	10702,	10740,   //  10 -  19 
	10779,	10818,	10857,	10896,	10935,	10973,	11012,	11051,	11090,	11129,   //  20 -  29 
	11167,	11206,	11245,	11283,	11322,	11361,	11400,  11438,	11477,	11515,   //  30 -  39 
	11554,	11593,	11631,	11670,	11708,	11747,	11786,	11824,	11863,	11901,   //  40 -  49 
	11940,	11978,	12017,	12055,	12094,	12132,	12171,	12209,	12247,	12286,   //  50 -  59 
	12324,	12363,	12401,	12439,	12478,	12516,	12554,	12593,	12631,	12669,   //  60 -  69 
	12708,	12746,	12784,	12822,	12861,	12899,	12937,	12975,	13013,	13052,   //  70 -  79 
	13090,	13128,	13166,	13204,	13242,	13280,	13318,	13357,	13395,	13433,   //  80 -  89 
	13471,	13509,	13547,	13585,	13623,	13661,	13699,	13737,	13775,	13813,   //  90 -  99 
	13851,	13888,	13926,	13964,	14002,	14040,	14078,	14116,	14154,	14191,   // 100 - 109 
	14229,	14267,	14305,	14343,	14380,	14418,	14456,	14494,	14531,	14569,   // 110 - 119 
	14607,	14644,	14682,	14720,	14757,	14795,	14833,	14870,	14908,	14946,   // 120 - 129 
	14983,	15021,	15058,	15096,	15133,	15171,	15208,	15246,	15283,	15321,   // 130 - 139 
	15358,	15396,	15433,	15471,	15508,	15546,	15583,	15620,	15658,	15695,   // 140 - 149 
	15733,	15770,	15807,	15845,	15882,	15919,	15956,	15994,	16031,	16068,   // 150 - 159 
	16105,	16143,	16180,	16217,	16254,	16291,	16329,	16366,	16403,	16440,   // 160 - 169 
	16477,	16514,	16551,	16589,	16626,	16663,	16700,  16737,	16774,	16811,   // 170 - 179 
	16848,	16885,	16922,	16959,	16996,	17033,	17070,	17107,	17143,	17180,   // 180 - 189 
	17217,	17254,	17291,	17328,	17365,	17402,	17438,	17475,	17512,	17549,   // 190 - 199 
	17586}; /* 200 */












































































































































































































































































































































































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

