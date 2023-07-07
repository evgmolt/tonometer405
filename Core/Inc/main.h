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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#define DEBUG

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
#define SEND_HTTP 7
#define SEND_SAVE_BUFF_MSG 8

#define STOP_MEAS_LEVEL 60
#define MIN_PRESSURE 130    
#define MAX_ALLOWED_PRESSURE 182
#define SUPER_MAX_ALLOWED_PRESSURE 250
#define SEC_AFTER_MAX 8
#define DELAY_AFTER_PUMPING 50
#define DELAY_AFTER_START 400
#define DELAY_FOR_ERROR 1000
#define PRESSURE_FOR_ERROR 16

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
#define SIM800_BUFFER_SIZE 1000

#define DERIVATIVE_SHIFT 13
#define DERIVATIVE_AVER_WIDTH 4

#define SERIAL_NUM_SIZE 13
#define BLE_PACKET_SIZE 20

//команды BLE
#define BLE_CMD_DATETIME    3
#define BLE_CMD_SERIAL      4

#define BLE_CMD_SET         6  //Нужно прибавить соответствующее F_XXX
#define BLE_CMD_GET         12 //Нужно прибавить соответствующее F_XXX

#define F_URL      0
#define F_PORT     1
#define F_LOGIN    2
#define F_PASSWORD 3
#define F_POINT    4
#define F_ID       5

#define EEPROM_CELL_SIZE 0xFF
#define EEPROM_BASE_ADDR 0xA0
#define EEPROM_SERIAL_ADDR (BLE_CMD_GET + F_ID + 1) * EEPROM_CELL_SIZE
#define EEPROM_RATE_ADDR EEPROM_SERIAL_ADDR + SERIAL_NUM_SIZE

//Биты статуса
#define BODY_MOVE   1
#define CUFF_FIT    2
#define IRREG_PULSE 4
#define PULSE_RANGE 8
#define MEAS_POS    16

#define BLE_CONNECT 1
#define BLE_DISCONNECT 0

void TFT_print(void);

void SIM_recieve_OK(void);
void PrintNum(int16_t num, uint16_t X0, uint16_t Y0, uint8_t color);

void DeviceOff(void);
void Calibration(void);

#ifdef DEBUG
uint8_t USBSendSave(int16_t *mass1, int16_t *mass2);
#endif

void ClearScreen(void);
void usb_send_16(short int T1, short int T2);
void PrintError(uint8_t K);
void BootMode(void);
void PrintBattCharge(void);
void BluetoothCheck(void);
uint8_t finder(uint8_t *buff, uint8_t *_string, uint8_t _char, uint16_t *num);
uint8_t BLECommandsReceiver(uint8_t *buff);

void SendSerialAT(uint8_t *serial_buf);
void GetNum();

void WriteBackupRegister(uint16_t day, uint16_t month, uint16_t year);
void CheckBackupRegister(uint16_t *_day, uint16_t *_month, uint16_t *_year);
void SendMeasurementResult(int16_t sis, int16_t dia, int16_t pressure, int16_t bonus);

void PrintSYS(int16_t IN);
void PrintDIA(int16_t IN);
void ResetBLEReceiver();
void AbortMeas(void);
void CreateJSON(void);
void SendResultAT(void);
void StopPumping(void);

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

extern uint8_t uart1_buff[200];
extern uint8_t uart1_count;

extern uint8_t uart2_buff[200];
extern uint8_t uart2_count;

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
extern bool show_heart;
extern bool erase_heart;
extern int16_t silence_time_start;
extern int16_t puls_buff[50];
extern uint8_t puls_counter;

extern uint16_t pulse;

extern uint32_t *ptrd;
extern uint32_t address;
extern uint8_t serial_num_string[SERIAL_NUM_SIZE];
/* calculate the number of page to be programmed/erased */
extern uint32_t page_num;

extern int16_t puls_buff_NEW[50];
extern int16_t puls_buff_AMP[50];
extern int16_t puls_buff_AMP_MIN[50];
extern int16_t puls_buff_IND_MIN[50];

extern uint16_t frequency;

extern int16_t p_sys;
extern int16_t p_dia;
extern int16_t p_mean;

extern int index_p_sys;
extern int index_p_dia;
extern int16_t x_max;

extern int16_t current_pressure;

extern int16_t i2c_out;
extern int zero_value;
extern uint8_t indicate_charge_toggle;
extern uint8_t indicate_charge_counter;

extern uint8_t bluetooth_status;
extern uint8_t status_byte;

extern int16_t dc_array_window;
extern int16_t ac_array_window;
extern uint8_t UART0_flag;

extern uint8_t mode;

extern bool ble_data_ready;
extern uint8_t ble_buffer_counter;
extern uint8_t sim800_FLAG;
extern uint8_t rang_batt_old;

extern uint8_t send_buff[100];

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

extern uint8_t sim800_buffer[SIM800_BUFFER_SIZE];
extern uint16_t sim800_buf_counter;
extern uint8_t brace_counter;

extern int lock_counter;

extern float rate;
extern float rate_whole;
extern float rate_fract;

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
