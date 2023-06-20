/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "ADS1115.h"
#include "Display.h"
#include "DataProcessing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t pulse = 0;

uint32_t *ptrd;
uint32_t address = 0x00000000U;
uint8_t SERIAL[SERIAL_NUM_SIZE]   = {'T','O','N','0','2','0','2','2','2','0','0','1',0};
/* calculate the number of page to be programmed/erased */


int16_t puls_buff_NEW[50]={0};
int16_t puls_buff_AMP[50]={0};
int16_t puls_buff_AMP_MIN[50]={0};
int16_t puls_buff_IND_MIN[50]={0};

uint16_t frequency=128;

uint16_t Lo_thresh_default = 0x8000;
uint16_t Hi_thresh_default = 0x0000;

uint8_t Hi_ADS1115_config = 0b10001111;
uint8_t Lo_ADS1115_config = 0b11100100;

uint8_t ADS1115_FLAG=0;

int16_t PSys = 0;
int16_t PDia = 0;
int16_t PMean;
uint16_t m_ss;
uint16_t m_mm;
uint16_t m_hh;
int indexPSys = 0;
int indexPDia = 0;
int16_t XMax;
int16_t current_pressure=0;
int16_t i2c_out=0;
int ZeroVal=0;
uint8_t indicate_charge_toggle=1;
uint8_t indicate_charge_counter=1;
uint16_t cur_day=13, cur_month=12, cur_year=2022;
uint32_t cur_thh=0,cur_tmm=0,cur_tss=0;
uint32_t cur_time=0;
uint8_t bluetooth_status=0;
uint8_t status_byte=0;
uint8_t mode = INIT_START;
bool ble_data_ready = false;
uint8_t ble_buffer_counter = 0;
uint8_t sim800_FLAG=0;
uint8_t rang_batt_old=99;
uint8_t i2c_transmitter[16];
uint8_t i2c_receiver[16];
uint8_t send_buff[100]={0};
uint8_t buff097[10]={0};
//usb_dev usbd_cdc;
uint16_t adc_value[8];
uint16_t num_string=0;
uint16_t count_send_bluetooth=0;
short int ble_buffer[BLE_PACKET_SIZE] = {0};
short int pressure_array[MAIN_ARRAY_SIZE]={0};
uint32_t main_index = 0;
uint32_t first_max;
uint32_t total_size = 0;
short int pressure_pulsation_array[MAIN_ARRAY_SIZE]={0};
short int envelope_array[MAIN_ARRAY_SIZE]={0};
uint32_t send_counter=0;

int lock_counter = 0;

uint8_t usb_command;

double rate=18.1;
double rate_whole;
double rate_fract;

bool arrhythmia = false;
bool stop_meas = false;
bool overpumping = false;

uint8_t num_of_packet = 0;
uint8_t current_packet_num = 0;
uint8_t command;
uint8_t byte_num = 0;
uint8_t result_index = 0;
uint8_t index_in_packet = 0;
uint8_t checksum = 0;    
uint8_t view_time = 1;

uint8_t mean_interval;

uint8_t get_number_mode = 0;
uint8_t allow_send_data = 0;
uint8_t send_serial_now = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_HCD_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();
  InitADC();
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        HAL_IWDG_Refresh(&hiwdg);
        switch (mode)
        {
            case INIT_START:
                HAL_TIM_Base_Start_IT(&htim2);
                if (!button_pressed) 
                {
                    mode = START_SCREEN;
//                    mode = PRESSURE_TEST;
                    allow_send_data = 0;
                    button_released = 0;
                    button_pressed_counter = 0;                    
                }
                break;
            case START_SCREEN:
                BluetoothCheck();
                TFT_print();
// Переход в режим зарядки при подключении USB разъема      
#ifndef DEBUG            
                if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10))
                {
                    ILI9341_FillScreen(ILI9341_BLACK);
                    print_battery();
                    mode = USB_CHARGING;
                }
#endif
                if (button_released) 
                {
                    view_time = 0;
                    send_buf_UART_1("AT\r", 3);            
                    
                    ILI9341_FillRectangle(0, 0, 240, 280, ILI9341_WHITE);                            
                    ILI9341_FillRectangle(100, 270, 140, 50, ILI9341_WHITE);                        
                
                    count_send_bluetooth=0;
                
                    current_pressure=0;
                    Calibration();
                    PUMP_ON;
                    VALVE_FAST_CLOSE;
                    VALVE_SLOW_CLOSE;
                    lock_interval=50;
                    ResetDetector();
                    puls_counter=0;            
//                    Timer2Start();
                    stop_meas = false;
                    mode = PUMPING_MANAGEMENT;
                    button_released = 0;
                    button_pressed_counter = 0;
                }
                break;
            case KEY_OFF:
                DeviceOff();
                break;
            case PUMPING_MANAGEMENT:
                BluetoothCheck();
                shutdown_counter = 0;
                ILI9341_FillRectangle(65, 245, 45, 27, ILI9341_WHITE);
                
                if (button_released)
                {
                    if (current_pressure < MAX_ALLOWED_PRESSURE) overpumping = false;
                    if (overpumping) 
                    {
                        StopPumping();
                        break;
                    }
                    else AbortMeas();
                }
                if (show_pressure_counter == 0)
                {
                    show_pressure_counter = SHOW_PRESSURE_INTERVAL;
                    if (current_pressure >= 0 & current_pressure<400) PrintNum(current_pressure, BIG_NUM_RIGHT, DIA_TOP, GREEN);
                }
                 
                if (!overpumping)
                {
                    if (current_pressure >= MAX_ALLOWED_PRESSURE && process_counter > MIN_PUMPING_INTERVAL) 
                    {
                        StopPumping();
                    }    
                }                
                if (current_pressure >= SUPER_MAX_ALLOWED_PRESSURE) 
                {
                    StopPumping();
                }    
                break;
            case USB_CHARGING:
                shutdown_counter = 0;
                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)==0) indicate_charge_toggle=1; //Не работает
                PrintBattCharge();                
                delay_1ms(1500);                
                if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)==0) DeviceOff();                        
                break;
            case PRESSURE_TEST:
                shutdown_counter = 0;
                GetADCData(false);
                PrintNum(current_pressure, BIG_NUM_RIGHT, DIA_TOP, GREEN);
                if (allow_send_data == 1) usb_send_16(i2c_out,0);
                delay_1ms(200);
                PrintTime(rtc_counter_get());
                ILI9341_WriteString(TIME_LEFT, TIME_TOP + 25, SERIAL, Font_Arial, ILI9341_BLACK, ILI9341_WHITE);          
                if (usb_command == USB_COMMAND_SET_RATE)
                {   
                    rate = rate_whole + rate_fract / 100;
                }
                break;
            case MEASUREMENT:
                shutdown_counter = 0;
                if (button_released) 
                {
                    if (current_pressure < MAX_ALLOWED_PRESSURE + 5) AbortMeas();
                    button_pressed_counter = 0;
                    button_released = 0;
                    overpumping = false;
                }
                if (show_pressure_counter == 0)
                {
                    show_pressure_counter = SHOW_PRESSURE_INTERVAL;
                    if (current_pressure>=0 & current_pressure<400) PrintNum(current_pressure, BIG_NUM_RIGHT, DIA_TOP, GREEN);
                }
                if (ble_data_ready)
                {  
                    if (count_send_bluetooth < 2)
                    {
                        ble_data_ready = false;
                        count_send_bluetooth++;
                    }
                    else
                    {
                        ble_data_ready = false;
                        uint8_t c_summ=0;                            
                        uint8_t cur_buff_ble[400]={'0', '2', 0x05, (count_send_bluetooth - 2) & 0xFF, ((count_send_bluetooth - 2) >> 8) & 0xFF, BLE_PACKET_SIZE};
                        
                        for (int f = 0; f < BLE_PACKET_SIZE; f++)
                        {
                            int16_t cur_press = ble_buffer[f];
                            cur_buff_ble[6 + f * 2] = cur_press & 0xFF;
                            cur_buff_ble[6 + f * 2 + 1] = (cur_press >> 8) & 0xFF;
                        }                            
                        for (int f=0; f < BLE_PACKET_SIZE * 2 + 6; f++)
                        {
                            c_summ+=cur_buff_ble[f];
                        }
                        cur_buff_ble[BLE_PACKET_SIZE * 2 + 6] = c_summ;
                        send_buf_UART_0(cur_buff_ble, BLE_PACKET_SIZE * 2 + 6 + 1);
                        count_send_bluetooth++;
                    }
                }
                if (current_pressure <= STOP_MEAS_LEVEL || stop_meas)
                {
                    total_size = main_index;
                    current_max=0;        
                    global_max=0;        
                    detect_level=detect_level_start;
                    silence_time_start=0;
                    puls_counter=0;            

                    for (int i = 0; i < total_size; i++)
                    {
                        Detect(first_max, i);
                    }
                    
                    ILI9341_FillRectangle(SYS_DIA_LEFT, SYS_TOP, 180, 106, ILI9341_WHITE);
                    ILI9341_FillRectangle(SYS_DIA_LEFT, DIA_TOP, 180, 106, ILI9341_WHITE);
                    ILI9341_FillRectangle(PULSE_LEFT, PULSE_TOP, 123, 64, ILI9341_WHITE);    
                
                    memset(envelope_array, 0, MAIN_ARRAY_SIZE);
                    GetArrayOfWaveIndexes(pressure_pulsation_array, puls_buff, puls_buff_NEW);
                    f_sorting_MAX();
                    CountEnvelopeArray(puls_buff_NEW,puls_buff_AMP);
                    
                    pulse = CountPulse();    
                    GetSysDia();
                    status_byte=0;
                    if (PSys > MIN_SYS & 
                        PSys < MAX_SYS & 
                        PDia > MIN_DIA & 
                        PDia < MAX_DIA & 
                        pulse > MIN_PULSE & 
                        pulse < MAX_PULSE) 
                    {
                        PrintSYS_label(true);
                        PrintDIA_label(true);    
                        PrintSYS(PSys);
                        PrintDIA(PDia);                                    
                        PrintNum((int16_t)pulse, BIG_NUM_RIGHT, PULSE_TOP, BLACK);

                        if (arrhythmia)
                        {
                            PrintHeartX3(true);
                            status_byte |= IRREG_PULSE;
                        }
                        SendATCommand();
                    
                        cur_time = rtc_counter_get();
                        m_hh = cur_time / 3600;
                        m_mm = (cur_time % 3600) / 60;
                        m_ss = (cur_time % 3600) % 60;
                        CheckBackupRegister(&cur_day, &cur_month, &cur_year);
                        if     (cur_year>=255)    cur_year-=2000;
                    }
                    else 
                    {
                        status_byte|=0x80;
                        PrintError(ERROR_MEAS);                            
                    }
                    SendMeasurementResult((uint8_t)cur_day, (uint8_t)cur_month, (uint8_t)cur_year, (uint8_t)m_ss, (uint8_t)m_mm, (uint8_t)m_hh, (uint8_t)PSys, (uint8_t)PDia, (uint8_t)pulse,status_byte);
                    
                    VALVE_FAST_OPEN;
                    VALVE_SLOW_OPEN;
#ifdef DEBUG                    
                    mode = SEND_SAVE_BUFF_MSG;   
#else                   
                        mode = INIT_START;
#endif                    
                }                    
                break;
            case SEND_SAVE_BUFF_MSG:
                shutdown_counter = 0;
                break;
        }
        
        if (UART0_count > 0)
        {                    
           BLECommandsReceiver(UART0_buff);
        }
        
        if (show_heart)
        {             
            PrintHeart(true);
            show_heart = false;
        }
        
        if (erase_heart)
        {
            PrintHeart(false);
            erase_heart = false;
        }
                    
/*        if (0U == cdc_acm_check_ready(&usbd_cdc)) 
        {
            cdc_acm_data_receive(&usbd_cdc);
        } 
        else 
        {
            cdc_acm_data_send(&usbd_cdc);
        }            */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t BLECommandsReceiver(uint8_t *buff)
{
    const uint8_t top = 20;
    const uint8_t left = 20;
    const uint8_t step = 35;
    uint8_t timestr[20]={0};
    
    if (get_number_mode) 
    {
        get_number_mode = 0;
        for (int i = 0; i < UART0_count; i++)
        {
            SERIAL[i] = buff[i];
        }
        SERIAL[UART0_count - 1] = 0;
        UART0_count = 0;
        return 0;
    }

    for (int i = 0; i < UART0_count; i++)
    {
        checksum += buff[i];
        switch (byte_num)
        {
            case 0:
                if (buff[i] == '0') byte_num = 1;
                checksum = buff[i];
                index_in_packet = 0;
                break;
            case 1:
                if (buff[i] == '2') byte_num = 2;
                break;
            case 2:
                command = buff[i];
                byte_num = 3;
                break;
            case 3:
                num_of_packet = buff[i];
                current_packet_num++;
                byte_num = 4;
                break;
            case 4:
                byte_num = 5;
                break;
        }
        if (byte_num == 5)
        {
            send_buff[result_index] = buff[i];
            result_index++;
            if (command == BLE_CMD_SERIAL) 
            {
                if (result_index == 7) //длина номера без '0' и '2'
                {
                    if (checksum == buff[i + 1])
                    {
                        for (uint8_t j = 0; j < 7; j++)
                        {
                            SERIAL[j + 2] = send_buff[j];
                        }
//                        FmcSerialSendAT();                        
                        delay_1ms(200);                                    
                        DeviceOff();
                    }
                }
            }
            if (command == BLE_CMD_DATETIME)
            {
                if (result_index == 6) //длина буфера
                {
                    if (checksum == buff[i + 2])
                    {
                        cur_year = send_buff[0];
                        cur_month = send_buff[1];
                        cur_day = send_buff[2];
                        cur_thh = send_buff[3];
                        cur_tmm = send_buff[4];
                        cur_tss = send_buff[5];
                        TimeSet((uint32_t)cur_thh, (uint32_t)cur_tmm, (uint32_t)cur_tss);
                        WriteBackupRegister((uint16_t)cur_day, (uint16_t)cur_month, (uint16_t)cur_year);
                        sprintf(timestr, "%02d:%02d:%02d  %02d.%02d.20%d", cur_thh, cur_tmm, cur_tss, cur_day, cur_month, cur_year);
                        ILI9341_WriteString(TIME_LEFT, TIME_TOP, timestr, Font_Arial, ILI9341_RED, ILI9341_WHITE);  
                        ResetBLEReceiver();
                        UART0_count = 0;
                        return 1;
                    }
                }                
            }
            if (command >= BLE_CMD_SETURL && command <= BLE_CMD_SETID)
            {
                if (index_in_packet == BLE_PACKET_SIZE - 2)
                {
                    index_in_packet = 0;
                    byte_num = 0;
                }
                if (buff[i] == 0)
                {
                    if (checksum == buff[i + 1])
                    {
                        ILI9341_WriteString(left, step / 2 + (command - 6) * step, send_buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);  
                        ResetBLEReceiver();
                    }
                }
            }
            if (command >= BLE_CMD_GETURL && command <= BLE_CMD_GETID)
            {
                if (buff[i + 1] == checksum) 
                {
                    sprintf(send_buff,"Something");
                    send_buf_UART_0(send_buff, 10);
                    ResetBLEReceiver();
                }
            }
        }
        index_in_packet++;
    }
    UART0_count = 0;
}

void ResetBLEReceiver()
{
    checksum = 0;
    byte_num = 0;
    index_in_packet = 0;
    current_packet_num = 0;
    result_index = 0;
}

void SendMeasurementResult(uint8_t c_day, uint8_t c_month, uint8_t c_year, uint8_t c_ss, uint8_t c_mm, uint8_t c_hh, int16_t sis, int16_t dia, int16_t pressure, int16_t bonus)
{
    uint8_t cur_buff[13]={'0','2', 0x01, c_day, c_month, c_year, c_ss, c_mm, c_hh, sis, dia, pressure, bonus};        
    uint8_t c_summ=0;        
    for (int q = 0; q < 13; q++)
    {
       c_summ += cur_buff[q];
    }            
    cur_buff[13] = c_summ;
    send_buf_UART_0(cur_buff, 14);
}

void SendATCommand()
{
    uint8_t len;
    uint8_t buff[50] = {0};
    sprintf(buff, "AT+RESULT=%02d.%02d.%02d_%02d:%02d:%d,%d,%d,%d,%d,%d\n", 5, 4, 2023, 8, 5, 30, PSys, PDia, PMean, pulse, status_byte);
    len = strlen(buff);
    send_buf_UART_0(buff, len);
}

void send_buf_UART_0(uint8_t *buf, uint8_t num) //Передача данных по BLE
{
    for(int j1=0; j1 < num; j1++)
    {
//        usart_data_transmit(USART0, (uint8_t)buf[j1]);
//        while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));            
    }
}

void send_buf_UART_1(uint8_t *buf, uint8_t num)
{
    for(int j1=0;j1<num;j1++)
    {
//        usart_data_transmit(USART1, (uint8_t)buf[j1]);
//        while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));            
    }
}

void StopPumping()
{
    ResetDetector();
    puls_counter=0;      
    stop_meas = false;
    PUMP_OFF;
    VALVE_SLOW_OPEN;
    count_send_bluetooth = 0;
    mode = MEASUREMENT;        
}

void AbortMeas(void) 
{
    PUMP_OFF;
    VALVE_FAST_OPEN;
    VALVE_SLOW_OPEN;
    button_released = 0;
    button_pressed_counter = 0;
    mode = START_SCREEN;
    PrintNum(0, BIG_NUM_RIGHT, DIA_TOP, GREEN);
}

void usb_send_16(short int T1, short int T2)
{
    uint8_t send_H1 = (T1 >> 8) & 0xFF;
    uint8_t send_L1 = T1 & 0xFF;
    uint8_t send_H2 = (T2 >> 8) & 0xFF;
    uint8_t send_L2 = T2 & 0xFF;
    uint8_t send_buff[5] = {25, send_L1, send_H1, send_L2, send_H2};
//    usbd_ep_send (&usbd_cdc, CDC_IN_EP, send_buff, 5);
}

void DeviceOff(void)
{
    PUMP_OFF;
    VALVE_FAST_OPEN;
    VALVE_SLOW_OPEN;
    ILI9341_FillScreen(ILI9341_BLACK);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

void BluetoothCheck()
{
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
