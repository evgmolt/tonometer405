/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC13_S_VALVE_O_C_Pin GPIO_PIN_13
#define PC13_S_VALVE_O_C_GPIO_Port GPIOC
#define PC1_SIM_PWKEY_Pin GPIO_PIN_1
#define PC1_SIM_PWKEY_GPIO_Port GPIOC
#define PC2_LCD_CS_Pin GPIO_PIN_2
#define PC2_LCD_CS_GPIO_Port GPIOC
#define PC3_LCD_NRST_Pin GPIO_PIN_3
#define PC3_LCD_NRST_GPIO_Port GPIOC
#define PA0_SIM_NETLIGHT_Pin GPIO_PIN_0
#define PA0_SIM_NETLIGHT_GPIO_Port GPIOA
#define PA1_SIM_STATUS_Pin GPIO_PIN_1
#define PA1_SIM_STATUS_GPIO_Port GPIOA
#define PA2_SIM_RX_MCU_TX_Pin GPIO_PIN_2
#define PA2_SIM_RX_MCU_TX_GPIO_Port GPIOA
#define PA3_SIM_TX_MCU_RX_Pin GPIO_PIN_3
#define PA3_SIM_TX_MCU_RX_GPIO_Port GPIOA
#define PA_5_LCD_SCK_Pin GPIO_PIN_5
#define PA_5_LCD_SCK_GPIO_Port GPIOA
#define PA6_LCD_MISO_Pin GPIO_PIN_6
#define PA6_LCD_MISO_GPIO_Port GPIOA
#define PA7_LCD_MOSI_Pin GPIO_PIN_7
#define PA7_LCD_MOSI_GPIO_Port GPIOA
#define PC4_LCD_DC_RS_Pin GPIO_PIN_4
#define PC4_LCD_DC_RS_GPIO_Port GPIOC
#define PB0_POWER_ON_DET_Pin GPIO_PIN_0
#define PB0_POWER_ON_DET_GPIO_Port GPIOB
#define PB1_VBAT_MESH_Pin GPIO_PIN_1
#define PB1_VBAT_MESH_GPIO_Port GPIOB
#define PB10_ADC_RDY_Pin GPIO_PIN_10
#define PB10_ADC_RDY_GPIO_Port GPIOB
#define PB11_MESH_OP_Pin GPIO_PIN_11
#define PB11_MESH_OP_GPIO_Port GPIOB
#define PC8_BTN_MCU_EN_Pin GPIO_PIN_8
#define PC8_BTN_MCU_EN_GPIO_Port GPIOC
#define PC9_MCU_3V3_EN_Pin GPIO_PIN_9
#define PC9_MCU_3V3_EN_GPIO_Port GPIOC
#define PA9_HC_RX_MCU_TX_Pin GPIO_PIN_9
#define PA9_HC_RX_MCU_TX_GPIO_Port GPIOA
#define PA10_HC_TX_MCU_RX_Pin GPIO_PIN_10
#define PA10_HC_TX_MCU_RX_GPIO_Port GPIOA
#define PA11_0D__Pin GPIO_PIN_11
#define PA11_0D__GPIO_Port GPIOA
#define PA_12_0D__Pin GPIO_PIN_12
#define PA_12_0D__GPIO_Port GPIOA
#define PA13_SWDIO_Pin GPIO_PIN_13
#define PA13_SWDIO_GPIO_Port GPIOA
#define PA14_SWCLK_Pin GPIO_PIN_14
#define PA14_SWCLK_GPIO_Port GPIOA
#define BT_STAT_Pin GPIO_PIN_15
#define BT_STAT_GPIO_Port GPIOA
#define PC10_USB_ON_Pin GPIO_PIN_10
#define PC10_USB_ON_GPIO_Port GPIOC
#define PC11_PUMP_ON_OFF_Pin GPIO_PIN_11
#define PC11_PUMP_ON_OFF_GPIO_Port GPIOC
#define PC12_F_VALVE_O_C_Pin GPIO_PIN_12
#define PC12_F_VALVE_O_C_GPIO_Port GPIOC
#define PB6_ADC_SCL_Pin GPIO_PIN_6
#define PB6_ADC_SCL_GPIO_Port GPIOB
#define PB7_ADC_SDA_Pin GPIO_PIN_7
#define PB7_ADC_SDA_GPIO_Port GPIOB
#define PB8_STDBY_Pin GPIO_PIN_8
#define PB8_STDBY_GPIO_Port GPIOB
#define PB9_CHRG_Pin GPIO_PIN_9
#define PB9_CHRG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define GREEN   0x1
#define RED     0x2
#define YELLOW  0x3
#define BLACK   0x4

#define YELLOW_LEVEL_SYS 130
#define RED_LEVEL_SYS    140
#define YELLOW_LEVEL_DIA 85
#define RED_LEVEL_DIA    90

#define PUMP_ON             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)
#define PUMP_OFF            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)
#define VALVE_FAST_CLOSE    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define VALVE_FAST_OPEN     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define VALVE_SLOW_CLOSE    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define VALVE_SLOW_OPEN     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)

#define SIM800_PWRKEY_DOWN  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
#define SIM800_PWRKEY_UP    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define SIM800_EXT_DOWN     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define SIM800_EXT_UP       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)

#define INIT_START 0
#define START_SCREEN 1
#define KEY_OFF 2
#define PUMPING_MANAGEMENT 3
#define USB_CHARGING 4
#define PRESSURE_TEST 5
#define MEASUREMENT 6
#define SEND_SAVE_BUFF_MSG 7

#define STOP_MEAS_LEVEL 60
#define MIN_PRESSURE 130    
#define MAX_ALLOWED_PRESSURE 182
#define SUPER_MAX_ALLOWED_PRESSURE 250
#define SEC_AFTER_MAX 8
#define DELAY_AFTER_PUMPING 50
#define DELAY_AFTER_START 400
#define DELAY_FOR_ERROR 1000
#define PRESSURE_FOR_ERROR 12

#define DEBONCE_INTERVAL 4
#define GO_TO_TEST_INTERVAL 500
#define SWITCH_OFF_INTERVAL 120
#define SHUTDOWN_INTERVAL 10000 //100 seconds
#define SEVEN_SECONDS 700
#define MIN_PUMPING_INTERVAL 1000
#define SHOW_PRESSURE_INTERVAL 40

#define CONNECTED 1
#define DISCONNECTED 0

#define MAX_SYS 250
#define MIN_SYS 60
#define MAX_DIA 150
#define MIN_DIA 30
#define MIN_PULSE 25
#define MAX_PULSE 250

#define LOCK_INTERVAL 20
#define HEART_INTERVAL 20

#define USB_COMMAND_SET_RATE   11
#define USB_COMMAND_GET_SERIAL 12

#define ERROR_CUFF 2
#define ERROR_TIME 3
#define ERROR_MEAS 4

#define NO_WAVE_INTERVAL 250

//Координаты
#define LEFT_MIN 5
#define BIG_NUM_RIGHT 230
#define HEART_LEFT 70
#define HEART_TOP 279
#define GSM_LEFT 22
#define GSM_TOP 258
#define HEART_X3_LEFT 61
#define HEART_X3_TOP 245
#define BLUETOOTH_TOP 255
#define SYS_DIA_LEFT 50
#define SYS_TOP 10
#define DIA_TOP 124
#define PULSE_LEFT 107
#define PULSE_TOP 240
#define TIME_TOP 20
#define TIME_LEFT 10

#define MAIN_ARRAY_SIZE 10000

#define DERIVATIVE_SHIFT 13
#define DERIVATIVE_AVER_WIDTH 4

#define SERIAL_NUM_SIZE 13
#define BLE_PACKET_SIZE 20

//команды BLE
#define BLE_CMD_DATETIME    3
#define BLE_CMD_SERIAL      4
//команды записи
#define BLE_CMD_SETURL      6
#define BLE_CMD_SETPORT     7
#define BLE_CMD_SETLOGIN    8
#define BLE_CMD_SETPASSWORD 9
#define BLE_CMD_SETPOINT    10
#define BLE_CMD_SETID       11
//команды чтения
#define BLE_CMD_GETURL      12
#define BLE_CMD_GETPORT     13
#define BLE_CMD_GETLOGIN    14
#define BLE_CMD_GETPASSWORD 15
#define BLE_CMD_GETPOINT    16
#define BLE_CMD_GETID       17

//Биты статуса
#define BODY_MOVE   1
#define CUFF_FIT    2
#define IRREG_PULSE 4
#define PULSE_RANGE 8
#define MEAS_POS    16

void i2c_config(void);
void my_delay(int time);    

void ADC_rcu_config(void);
void ADC_gpio_config(void);
void adc_config(void);
void dma_config(void);
void GPIO_config(void);
    
void spi_rcu_config(void);
void spi_gpio_config(void);    
void spi_config(void);

void nvic_configuration(void);
void rtc_configuration(void);
void PrintTime(uint32_t timevar);

void usb_send_i2c_convers(void);
void i2c_convers(void);    
void TFT_print(void);

void usart_config_0(void);
void send_buf_UART_0(uint8_t *buf, uint8_t num);

void usart_config_1(void);
void send_buf_UART_1(uint8_t *buf, uint8_t num);
void SIM_recieve_OK(void);
void PrintNum(int16_t num, uint16_t X0, uint16_t Y0, uint8_t color);
void TimeSet(uint32_t tmp_hh,uint32_t tmp_mm,uint32_t tmp_ss);
void TimeInit(void);
void ButtonInterruptConfig(void);

void DeviceOff(void);
void i2cCalibration(void);

void TimerConfig1(void);
void NvicConfig1(void);

void TimerConfig2(void);
void NvicConfig2(void);

void Timer2Start(void);
void Timer2Stop(void);
void Timer1Start(void);

short int convert_save_16(void);

uint8_t usb_send_save(int16_t *mass1, int16_t *mass2);

void ClearScreen(void);
void usb_send_16(short int T1, short int T2);
short int convert_NO_save(void);
void ADS1115_config(uint8_t pointer, uint8_t byte1, uint8_t byte2);
uint8_t ADS1115_read_IT(void);
void PrintError(uint8_t K);
void BootMode(void);
void PrintBattCharge(void);
void BluetoothCheck(void);
uint8_t finder(uint8_t *buff, uint8_t *_string, uint8_t _char, uint16_t *num);
uint8_t BLECommandsReceiver(uint8_t *buff);

/* erase fmc page from FMC_WRITE_START_ADDR */
void FmcErasePage(uint32_t page_address);
/* program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR */
/* check fmc erase result */
void FmcErasePage_check(void);
/* check fmc program result */
void FmcSerialSendAT(void);
void GetNum();

void WriteBackupRegister(uint16_t day, uint16_t month, uint16_t year);
void CheckBackupRegister(uint16_t *_day, uint16_t *_month, uint16_t *_year);
void SendMeasurementResult(uint8_t c_day, uint8_t c_month, uint8_t c_year, uint8_t c_ss, uint8_t c_mm, uint8_t c_hh, int16_t sis, int16_t dia, int16_t pressure, int16_t bonus);

void PrintSYS(int16_t IN);
void PrintDIA(int16_t IN);
void ResetBLEReceiver();
void AbortMeas(void);
void CreateJSON(void);
void SendATCommand(void);
void StopPumping(void);

double ReadRateFromFmc();

extern bool arrhythmia;
extern bool stop_meas;
extern bool overpumping;

extern uint8_t num_of_packet;
extern uint8_t current_packet_num;
extern uint8_t command;
extern uint8_t byte_num;
extern uint8_t checksum;    
extern uint8_t view_time;    

extern const int lo_limit; //ms - 200 
extern const int hi_limit; //ms - 30

extern uint8_t UART1_buff[200];
extern uint8_t UART1_count;

extern uint8_t UART0_buff[200];
extern uint8_t UART0_count;

extern int16_t detect_level_start;
extern double detect_level;
extern int16_t lock_interval;
extern double detect_levelCoeff;
extern double detect_levelCoeffDia;
extern double stop_meas_coeff;
extern int16_t current_interval;
extern double current_max;
extern double global_max;
extern uint8_t wave_detect_flag;
extern int16_t Wave_detect_time;
extern int16_t Wave_detect_time_OLD;
extern bool show_heart;
extern bool erase_heart;
extern int16_t silence_time_start;
extern int16_t puls_buff[50];
extern uint8_t puls_counter;

extern uint16_t pulse;

extern uint32_t *ptrd;
extern uint32_t address;
extern uint8_t SERIAL[SERIAL_NUM_SIZE];
/* calculate the number of page to be programmed/erased */
extern uint32_t page_num;

extern int16_t puls_buff_NEW[50];
extern int16_t puls_buff_AMP[50];
extern int16_t puls_buff_AMP_MIN[50];
extern int16_t puls_buff_IND_MIN[50];

extern uint16_t frequency;

extern uint16_t Lo_thresh_default;
extern uint16_t Hi_thresh_default;

extern uint8_t Hi_ADS1115_config;
extern uint8_t Lo_ADS1115_config;

extern uint8_t ADS1115_FLAG;

extern int16_t PSys;
extern int16_t PDia;
extern int16_t PMean;
extern uint16_t m_ss;
extern uint16_t m_mm;
extern uint16_t m_hh;

extern int indexPSys;
extern int indexPDia;
extern int16_t XMax;

extern int16_t current_pressure;

extern int16_t i2c_out;
extern int i2c_out_K;
extern uint8_t indicate_charge_toggle;
extern uint8_t indicate_charge_counter;

extern uint16_t cur_day, cur_month, cur_year;
extern uint32_t cur_thh,cur_tmm,cur_tss;
extern uint32_t cur_time;

extern uint8_t bluetooth_status;
extern uint8_t status_byte;

extern int16_t dc_array_window;
extern int16_t ac_array_window;
extern uint8_t UART0_flag;

extern uint8_t Hi_ADS1115_config;
extern uint8_t Lo_ADS1115_config;

extern uint8_t mode;

extern uint8_t ble_data_ready;
extern uint8_t ble_buffer_counter;
extern uint8_t sim800_FLAG;
extern uint8_t rang_batt_old;
extern uint8_t i2c_transmitter[16];
extern uint8_t i2c_receiver[16];

extern uint8_t send_buff[100];
extern uint8_t buff097[10];
extern uint16_t adc_value[8];

extern uint16_t num_string;

extern uint16_t count_send_bluetooth;

extern short int ble_buffer[BLE_PACKET_SIZE];
extern short int pressure_array[MAIN_ARRAY_SIZE];
extern uint32_t main_index;
extern uint32_t total_size;
extern uint32_t first_max;
extern short int pressure_pulsation_array[MAIN_ARRAY_SIZE];
extern short int envelope_array[MAIN_ARRAY_SIZE];

extern uint32_t send_counter;

extern int lock_counter;

extern double rate;
extern double rate_whole;
extern double rate_fract;

extern int shutdown_counter;
extern int process_counter;
extern int heart_counter;
extern int show_pressure_counter;

extern int button_touched;
extern int button_pressed;
extern int button_released;
extern int button_touched_counter;
extern int button_pressed_counter;

extern uint8_t usb_command;

extern uint8_t mean_interval;

extern uint8_t get_number_mode;
extern uint8_t allow_send_data;
extern uint8_t send_serial_now;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
