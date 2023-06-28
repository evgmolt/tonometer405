/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "ADS1115.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
const int lo_limit = 30;  // 256 bpm
const int hi_limit = 250; // 30 bpm

uint8_t uart1_buff[200]={0};
uint8_t uart1_count=0;

uint8_t uart2_buff[200]={0};
uint8_t uart2_count=0;

int16_t detect_level_start = 4;
double detect_level = 4;
int16_t lock_interval = 50;
double detect_levelCoeff = 0.7;
double detect_levelCoeffDia = 0.55;
double stop_meas_coeff = 0.58;
int16_t current_interval = 0;
double current_max=0;
double global_max=0;
uint8_t wave_detect_flag=0;
int16_t wave_detect_time=0;
int16_t wave_detect_time_old=0;
uint8_t wave_ind_flag=0;
bool show_heart = false;
bool erase_heart = false;
int16_t silence_time_start=0;
int16_t puls_buff[50]={0};
uint8_t puls_counter=0;

int16_t dc_array_window = 30;
int16_t ac_array_window = 4;

int shutdown_counter = 0;
int process_counter = 0;
int heart_counter = 0;
int show_pressure_counter = 0;

int button_touched = 0;
int button_pressed = 0;
int button_released = 0;
int button_touched_counter = 0;
int button_pressed_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */
    const int pwrkey_down_time = 100;
    const int ext_up_time = 120;
    const int pwrkey_up_time = 250;
    if (lock_counter > 0) lock_counter--;
    if (show_pressure_counter > 0) show_pressure_counter--;

    process_counter++;
/*        if (mode == INIT_START || mode == START_SCREEN || mode == PUMPING_MANAGEMENT) 
    {
        if (process_counter == pwrkey_down_time) 
        SIM800_PWRKEY_DOWN;
    if (process_counter == ext_up_time)      
        SIM800_EXT_UP;
    if (process_counter == pwrkey_up_time)   
        SIM800_PWRKEY_UP;
        if (process_counter == 300) 
            send_buf_UART_1("AT",strlen("AT"));            
    }*/
    
    if (heart_counter > 0)
    {
        heart_counter--;
        erase_heart = (heart_counter == 0);
    }
        
    shutdown_counter++;
    if (shutdown_counter > SHUTDOWN_INTERVAL) 
    {
        shutdown_counter = 0;
#ifndef DEBUG
        mode = KEY_OFF;                    
#endif
    }

    button_touched = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
    if (button_touched) 
    {
        button_touched_counter++;
        if (button_touched_counter > DEBONCE_INTERVAL) button_pressed = 1;
    }
    else 
    {
        button_touched = 0;
        button_pressed = 0;
        button_released = 0;
        button_touched_counter = 0;
    }
    
    if (button_pressed) 
    {
        button_pressed_counter++;
        shutdown_counter = 0;
        if (mode == PUMPING_MANAGEMENT) overpumping = true;
    }
    else 
    {
        if (button_pressed_counter > 0) button_released = 1;
    }

    if (mode == INIT_START) 
    {
        if (button_pressed_counter > GO_TO_TEST_INTERVAL) 
        {
            ILI9341_FillScreen(ILI9341_WHITE);
            HAL_TIM_Base_Stop_IT(&htim1);
            Calibration();
            VALVE_FAST_CLOSE;
            VALVE_SLOW_CLOSE;
            button_pressed_counter = 0;
            allow_send_data = 0;
            mode = PRESSURE_TEST;
        }
    }
    else 
    {
        if ((button_pressed_counter > SWITCH_OFF_INTERVAL) && (mode != USB_CHARGING) && (mode != PUMPING_MANAGEMENT)) 
        {
            if (!overpumping) mode = KEY_OFF;
        }
    }

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
    static int32_t max_index;
    
    int16_t current_value;

    if (mode == PUMPING_MANAGEMENT)
    {                                                                                                                                                                //////////////////////////////////
        if (GetADCData(true))
        {
            if (main_index<300) 
            {
//                convert_NO_save();
            }
            else if (main_index>300)
            {
                pressure_pulsation_array[main_index-1] = SmoothAndRemoveDC(pressure_array, dc_array_window, ac_array_window);											
            }	
        
            if (main_index >= DELAY_AFTER_START)
            {                                        
                current_value=GetDerivative(pressure_pulsation_array, main_index-1);
                usb_send_16(current_value,(short)current_max);
                if (current_value>current_max)
                {
                    current_max=current_value;
                    max_index=main_index;
                }
                if (current_pressure > MIN_PRESSURE && !overpumping)
                {                                                
                    if (main_index > max_index + SEC_AFTER_MAX * frequency)
                    {    
                        main_index=0;        
                        ble_buffer_counter = 0;
                        wave_detect_flag=0;                                                    
                        current_max = 0;    
                        global_max = 0;
                        Lock();
                        PUMP_OFF;
                        VALVE_SLOW_OPEN;
                        stop_meas = false;
                        count_send_bluetooth = 0;
                        mode = MEASUREMENT;
                    }
                }                                        
            }
            
            if (main_index > DELAY_FOR_ERROR & current_pressure < PRESSURE_FOR_ERROR)
            { 
                ResetDetector();
                HAL_TIM_Base_Stop_IT(&htim2);
                PrintError(ERROR_CUFF);
                PUMP_OFF;
                VALVE_FAST_OPEN;
                VALVE_SLOW_OPEN;
                mode = START_SCREEN;
            }
            
            if (main_index>9990)
            {
                ResetDetector();
                HAL_TIM_Base_Stop_IT(&htim2);
                PrintError(ERROR_TIME);
                PUMP_OFF;
                VALVE_FAST_OPEN;
                VALVE_SLOW_OPEN;
                mode = START_SCREEN;
            }
        }
    }
    else 
    if (mode == MEASUREMENT) 
    {
        if (GetADCData(true)) 
        {
            //ѕодготовка данных дл€ передачи по bluetooth
            ble_buffer_counter++;
            if (ble_buffer_counter > BLE_PACKET_SIZE - 1)
            {
                ble_buffer_counter = 0;
                for (int i = 0; i < BLE_PACKET_SIZE; i++)
                {
                    ble_buffer[i] = (pressure_array[main_index - 1 - BLE_PACKET_SIZE + i] * 100) / rate;                        
                }
                ble_data_ready = true;
            }
            
            if (main_index>100)
            {
                if (lock_counter > 0)
                {
                    pressure_pulsation_array[main_index-1] = 0; 
                }
                else
                {
                    pressure_pulsation_array[main_index-1] = SmoothAndRemoveDC(pressure_array, dc_array_window, ac_array_window);  
                }
            }    
            else         
            {
                pressure_pulsation_array[main_index-1]=0;
            }
                                
            if (main_index >= DELAY_AFTER_PUMPING)
            {                
                current_interval++;
                if (current_interval > NO_WAVE_INTERVAL)
                {
                    detect_level = detect_level_start;
                }
                current_value = GetDerivative(pressure_pulsation_array, main_index-1);
                usb_send_16(current_value, current_max); 
                if (current_value > detect_level & (main_index - 1) > (silence_time_start + lock_interval)) wave_detect_flag = 1;
                if (wave_detect_flag == 1 & (main_index-1)>(silence_time_start + lock_interval))
                {
                    if (current_value > current_max) 
                    {
                        current_max = current_value;
                        max_index = main_index - 1;
                    }
                    else if (current_value < detect_level)
                    {
                        global_max = fmax(global_max, current_max);
                        if (process_counter > SEVEN_SECONDS && current_max < global_max * stop_meas_coeff)
                        {
                            stop_meas = true;
                        }
                        first_max = max_index;
                        wave_detect_time_old = wave_detect_time;
                        wave_detect_time = max_index - 1;                                                                                                                        
                        puls_buff[puls_counter++]= max_index - 1;
                        heart_counter = HEART_INTERVAL;
                        show_heart = true;    
                        lock_interval = (wave_detect_time - wave_detect_time_old) / 2;
//                            if (lock_interval > hi_limit | lock_interval < lo_limit) lock_interval = 50;
                        silence_time_start = max_index - 1;
                        detect_level = current_max * detect_levelCoeff;
                        if (detect_level < detect_level_start) detect_level = detect_level_start;
                        current_max = 0;
                        current_interval = 0;
                        wave_detect_flag = 0;
                    }
                }                        
            }                        
        }
    }
#ifdef DEBUG        
    else if (mode == SEND_SAVE_BUFF_MSG) 
    {
        if (button_released) 
        {
            mode = START_SCREEN;
            button_released = 0;
            button_pressed_counter = 0;
            VALVE_FAST_OPEN;
            VALVE_SLOW_OPEN;
            PUMP_OFF;
        }
        else
        if (USBSendSave(pressure_pulsation_array,envelope_array))
        {            
            mode = INIT_START;
            HAL_TIM_Base_Stop_IT(&htim2);
            main_index=0;
            ble_buffer_counter = 0;
            send_counter=0;
        }
    } 
#endif       
    else if (mode == PRESSURE_TEST)
    {
        VALVE_FAST_CLOSE;
        VALVE_SLOW_CLOSE;
        if (send_serial_now)
        {
            send_serial_now = 0;
            CDC_Transmit_FS(serial_num_string, SERIAL_NUM_SIZE);
            allow_send_data = 1;
        }
    }

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void Lock(void)
{
    lock_counter = LOCK_INTERVAL;
}


bool GetADCData(bool save)
{
    HAL_StatusTypeDef result = GetADCValue();
    if (result != HAL_OK) return false;
    uint16_t current_value = ADS1115_value - zero_value;
    current_pressure = current_value / rate;
    RequestADC();
    if (!save) return true;
    pressure_array[main_index] = current_value;
    if (pressure_array[main_index] < 0) pressure_array[main_index] = 0;
    main_index++;        
    return true;
}

void Calibration()
{
    const uint8_t aver_size = 20;
    uint32_t sum = 0;
    uint8_t divisor = 0;
    for (int i = 0; i < aver_size; i++)
    {
        if (GetADCData(false)) 
        {
            sum += ADS1115_value;
            divisor++;
        }
    }
    zero_value = sum / divisor;
}

#ifdef DEBUG
uint8_t USBSendSave(int16_t *mass1, int16_t *mass2)
{
    //Add markers of SYS, MAX and DIA points into array
    for (int h = 0; h < puls_counter; h++)
    {
            if (send_counter == x_max) pressure_pulsation_array[send_counter] = 100;                    
    }        
    for (int h = 0; h < puls_counter; h++)
    {
            if (send_counter == index_p_sys | send_counter == index_p_dia) pressure_pulsation_array[send_counter] = -100;                    
    }        
    
    uint8_t send_H1 = (mass1[send_counter] >> 8) & 0xFF;
    uint8_t send_L1 = mass1[send_counter] & 0xFF;
    uint8_t send_H2 = (mass2[send_counter] >> 8) & 0xFF;
    uint8_t send_L2 = mass2[send_counter] & 0xFF;
    
    uint8_t send_buff[5] = {25, send_L1, send_H1, send_L2, send_H2};
    CDC_Transmit_FS(send_buff, 5);
    send_counter++;
    if (send_counter >= total_size) return 1;
    else return 0;
}
#endif

//‘ункци€ используетс€ при втором проходе по массиву пульсаций давлени€, когда уже известно положение максимума.
//ƒо максимума используетс€ detect_levelCoeff, а после detect_levelCoeffDia, так как скорость спада амплитуд пульсаций
//после максимума выше
void Detect(int32_t x_max, uint16_t index)
{                
    static int32_t index_of_max;
    int16_t current_value;
    double coeff;
        
    current_interval++;
    if (current_interval > NO_WAVE_INTERVAL)
    {
        detect_level = detect_level_start;
    }
    current_value = GetDerivative(pressure_pulsation_array, index - 1);
    if (current_value > detect_level & (index - 1) > (silence_time_start + lock_interval)) wave_detect_flag = 1;
    if (wave_detect_flag == 1 & (index - 1) > (silence_time_start + lock_interval))
    {
        if (current_value > current_max) 
        {
            current_max = current_value;
            index_of_max = index-1;
        }
        else if (current_value < detect_level)
        {
            global_max = fmax(global_max, current_max);
            wave_detect_time_old = wave_detect_time;
            wave_detect_time = index_of_max - 1;                                                                                                                        
            puls_buff[puls_counter++] = index_of_max-1;
            lock_interval=(wave_detect_time - wave_detect_time_old) / 2;
            silence_time_start = index_of_max - 1;
            if (index < x_max) coeff = detect_levelCoeff; else coeff = detect_levelCoeffDia;
            detect_level = current_max * coeff;
            if (detect_level < detect_level_start) detect_level = detect_level_start;
            current_max=0;
            current_interval = 0;
            wave_detect_flag=0;
        }
    }                        
}                        

void ResetDetector(void)
{
    process_counter = 0;
    main_index=0;        
    wave_detect_flag=0;    
    wave_detect_time = 0;
    wave_detect_time_old = 0;
    current_max=0;        
    global_max=0;        
    detect_level=detect_level_start;
    silence_time_start=0;
    ble_buffer_counter = 0;
}


/* USER CODE END 1 */
