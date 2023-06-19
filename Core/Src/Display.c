#include "main.h"
#include "Display.h"
#include "bat_dif_7_22.h"
#include "bat_clr_44_24.h"
#include "ili9341.h"
#include "text_H_GREEN.h"
#include "text_H_RED.h"
#include "text_H_YELLOW.h"
#include "text_L_BLACK.h"
#include "heart_31_30.h"
#include "heartX3_45_27.h"
#include "bluetooth_15_24.h"
#include "gsm_29_18.h"
#include "SYS_46_36.h"
#include "DIA_45_35.h"

#define BATT_RANG_MAX 2400
#define BATT_RANG_5 2364
#define BATT_RANG_4 2298
#define BATT_RANG_3 2232
#define BATT_RANG_2 2166
#define BATT_RANG_1 2100

//Max 2290

extern uint16_t adc_value[8];

void PrintHeart(bool show)
{
    if (show) 
    {
        ILI9341_DrawImage(HEART_LEFT, HEART_TOP, 31, 30, (const uint16_t*)heart);
    }
    else 
    {
        ILI9341_FillRectangle(HEART_LEFT, HEART_TOP, 31, 30, ILI9341_WHITE);    
    }
}

void PrintHeartX3(bool show)
{
    if (show) 
    {
        ILI9341_DrawImage(HEART_X3_LEFT, HEART_X3_TOP, 45, 27, (const uint16_t*)heartX3);
    }
    else 
    {
        ILI9341_FillRectangle(HEART_X3_LEFT, HEART_X3_TOP, 45, 27, ILI9341_WHITE);            
    }
}

void PrintBluetooth(bool show)
{    
    if (show) 
    {
        ILI9341_DrawImage(LEFT_MIN, BLUETOOTH_TOP, 15, 24, (const uint16_t*)bluetooth);
    }
    else 
    {
        ILI9341_FillRectangle(LEFT_MIN, BLUETOOTH_TOP, 15, 24, ILI9341_WHITE);            
    }
}

void PrintGsm(bool show)
{
    if (show) 
    {
        ILI9341_DrawImage(GSM_LEFT, GSM_TOP, 29, 18, (const uint16_t*)gsm);        
    }
    else 
    {
        ILI9341_FillRectangle(GSM_LEFT, GSM_TOP, 29, 18, ILI9341_WHITE);            
    }
}

void PrintSYS_label(bool show) 
{
    if (show) 
    {
        ILI9341_DrawImage(LEFT_MIN, SYS_TOP, 46, 36, (const uint16_t*)SYS);
    }
    else
    {
        ILI9341_FillRectangle(LEFT_MIN, SYS_TOP, 46, 36, ILI9341_WHITE);
    }
}

void PrintDIA_label(bool show) 
{
    if (show) 
    {
        ILI9341_DrawImage(LEFT_MIN, DIA_TOP, 45, 35, (const uint16_t*)DIA);
    }
    else
    {
        ILI9341_FillRectangle(LEFT_MIN, DIA_TOP, 45, 35, ILI9341_WHITE);
    }
}

void PrintSYS(int16_t IN)
{
    uint8_t color = GREEN;
    if (IN > YELLOW_LEVEL_SYS) color = YELLOW;
    if (IN > RED_LEVEL_SYS) color = RED;
    PrintNum(IN, BIG_NUM_RIGHT, SYS_TOP, color);
}

void PrintDIA(int16_t IN)
{
    uint8_t color = GREEN;
    if (IN > YELLOW_LEVEL_DIA) color = YELLOW;
    if (IN > RED_LEVEL_DIA) color = RED;
    PrintNum(IN, BIG_NUM_RIGHT, DIA_TOP, color);
}

void print_battery(void)
{
    ILI9341_FillRectangle(50, 150, 150, 70, ILI9341_BLACK);
    ILI9341_FillRectangle(52, 152, 146, 66, ILI9341_WHITE);
    ILI9341_FillRectangle(40, 150+35-20, 10, 40, ILI9341_BLACK);
    ILI9341_FillRectangle(40+2, 150+35-20+2, 10-2, 40-4, ILI9341_WHITE);
}

void PrintBattCharge(void)
{
    uint8_t num_of_segments = 6;
    uint16_t batt_level;

    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
    delay_1ms(100);        
    batt_level = adc_value[0] * 3300 / 0xFFF;

    uint8_t buff[5] = {0};
    sprintf(buff, "%04d:", batt_level);
//        ILI9341_WriteString(130, 230, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);

    if (batt_level > BATT_RANG_MAX) 
    {
        for (int i=1; i < num_of_segments; i++)
        {
           ILI9341_FillRectangle(200-i*29, 154, 25, 62, ILI9341_GREEN);                                    
        }
        return;
    }
    
    if(indicate_charge_toggle) 
    {
        indicate_charge_counter++;
        if (indicate_charge_counter > num_of_segments) 
        {
            indicate_charge_counter = 1;
            ILI9341_FillRectangle(52, 152, 146, 66, ILI9341_WHITE);
        }
        for (int i=1; i<indicate_charge_counter; i++){
                ILI9341_FillRectangle(200-i*29, 154, 25, 62, ILI9341_GREEN);                        
        }                    
        indicate_charge_toggle=0;
    }
    else 
    {
//                ILI9341_FillRectangle(52, 152, 146, 66, ILI9341_WHITE);
            indicate_charge_toggle=1;
    }
}

void PrintError(uint8_t K){
    ILI9341_FillRectangle(SYS_DIA_LEFT, SYS_TOP, 180, 106, ILI9341_WHITE);
    ILI9341_FillRectangle(SYS_DIA_LEFT, DIA_TOP, 180, 106, ILI9341_WHITE);
    ILI9341_FillRectangle(PULSE_LEFT, PULSE_TOP, 123, 64, ILI9341_WHITE);    

    uint8_t _buff[20]={0};
    uint8_t left_margin = 5;
    uint8_t top_margin = 30;
    
    if (K==ERROR_CUFF)
    {
        sprintf(_buff,"������ �������");        
        ILI9341_WriteString(left_margin, top_margin, _buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);            
        sprintf(_buff,"��������� �������");        
        ILI9341_WriteString(left_margin, top_margin + Font_Arial.height + 2, _buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);            
    }
    if (K==ERROR_TIME)
    {
        sprintf(_buff,"������ ���������");        
        ILI9341_WriteString(left_margin, top_margin, _buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);            
    }
    if (K==ERROR_MEAS)
    {
        sprintf(_buff,"������ %d %d %d", PSys, PDia, pulse);        
//        sprintf(_buff,"������ ���������");        
        ILI9341_WriteString(left_margin, top_margin, _buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);            
    }        
}

void ClearScreen(void)
{
    PrintHeart(false);
    PrintBluetooth(false);
    PrintGsm(false);
    PrintHeart(false);
    PrintHeartX3(false);
    PrintSYS_label(false);
    PrintDIA_label(false);
    ILI9341_FillRectangle(SYS_DIA_LEFT, SYS_TOP, 180, 106, ILI9341_WHITE);
    ILI9341_FillRectangle(SYS_DIA_LEFT, DIA_TOP, 180, 106, ILI9341_WHITE);
    ILI9341_FillRectangle(PULSE_LEFT, PULSE_TOP, 123, 64, ILI9341_WHITE);	
}

void PrintTime(uint32_t timevar)
{
    static uint32_t seconds;
    uint8_t buff[100]={0};
    uint32_t thh = 0, tmm = 0, tss = 0;
        
    // compute  hours 
    thh = timevar / 3600;
    // compute minutes 
    tmm = (timevar % 3600) / 60;
    // compute seconds 
    tss = (timevar % 3600) % 60;
    if (seconds == tss) return;
    seconds = tss;
        
        CheckBackupRegister(&cur_day, &cur_month, &cur_year);
        if (thh==0 & cur_day==0 & cur_month==0){

                cur_day=1;
                cur_month=1;
                WriteBackupRegister(cur_day, cur_month, cur_year);
        }
        
        if (thh>23) {
                cur_day++; 
                thh=0;
                TimeSet(thh,tmm,tss);
                WriteBackupRegister(cur_day, cur_month, cur_year);                
        }
        if (cur_day>=29) { //add calendar....
                cur_month++; 
                cur_day=1;    
                WriteBackupRegister(cur_day, cur_month, cur_year);
        }
        if (cur_month>12) {
                cur_year++; 
                cur_month=1;
                WriteBackupRegister(cur_day, cur_month, cur_year);
        }
        sprintf(buff, "%02d:%02d:%02d", thh, tmm, tss);
        ILI9341_WriteString(TIME_LEFT, TIME_TOP, buff, Font_Arial, ILI9341_BLACK, ILI9341_WHITE);  
        sprintf(buff, "%02d.%02d.20%d", cur_day, cur_month, cur_year);
        ILI9341_WriteString(TIME_LEFT + 85, TIME_TOP, buff, Font_Arial, ILI9341_BLACK, ILI9341_WHITE);  
}


void TFT_print(void)
{
    uint16_t batt_level=0;
    uint8_t rang_batt=0;

    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
    delay_1ms(100);        
    batt_level=adc_value[0] * 3300 / 0xFFF;

    uint8_t buff[20]={0};
//    sprintf(buff,"%04d:",batt_level);
    sprintf(buff, "�����");
    ILI9341_WriteString(10, 240, buff, Font_Arial, ILI9341_BLACK, ILI9341_WHITE);

    if (batt_level> BATT_RANG_MAX)                    rang_batt=5;
    else if (batt_level < BATT_RANG_MAX & batt_level > BATT_RANG_5) rang_batt=5;
    else if (batt_level < BATT_RANG_5 & batt_level > BATT_RANG_4) rang_batt=4;
    else if (batt_level < BATT_RANG_4 & batt_level > BATT_RANG_3) rang_batt=3;
    else if (batt_level < BATT_RANG_3 & batt_level > BATT_RANG_2) rang_batt=2;
    else if (batt_level < BATT_RANG_2 & batt_level > BATT_RANG_1) rang_batt=1;
    else if (batt_level < BATT_RANG_1)    rang_batt=0;
            
    if (rang_batt_old!=rang_batt)
    {    
        ILI9341_DrawImage(6, 285, 44, 24, (const uint16_t*)bat_clr);            
        for (int i1=0;i1<rang_batt;i1++)
        {
                ILI9341_DrawImage(42-i1*8, 286, 7, 22, (const uint16_t*)bat_dif);
        }
        rang_batt_old=rang_batt;
    }
    if (view_time)
    {
        PrintTime(rtc_counter_get());
        ILI9341_WriteString(TIME_LEFT, TIME_TOP + 25, SERIAL, Font_Arial, ILI9341_BLACK, ILI9341_WHITE);          
    }
}


void PrintNum(int16_t num, uint16_t X0, uint16_t Y0, uint8_t color)
{
    #define BIG_NUM_W 60
    #define MID_NUM_W 41
    #define BIG_NUM_H 106
    #define MID_NUM_H 64
    
    double now = 0;
    uint16_t now1 = 0;
    uint8_t max;
    if (num >= 100) max = 3;
    else 
    if (num >= 10) 
    {
        max = 2;
        if (color==BLACK) ILI9341_FillRectangle(X0 - MID_NUM_W * 3, Y0, MID_NUM_W, MID_NUM_H, ILI9341_WHITE);
        else  ILI9341_FillRectangle(X0 - BIG_NUM_W * 3, Y0, BIG_NUM_W, BIG_NUM_H, ILI9341_WHITE);
    }
    else 
    {
        max = 1;
        if (color==BLACK) ILI9341_FillRectangle(X0 - MID_NUM_W * 3, Y0, 82, MID_NUM_H, ILI9341_WHITE);
        else ILI9341_FillRectangle(X0 - BIG_NUM_W * 3, Y0, 120, BIG_NUM_H, ILI9341_WHITE);
    }
            
    for (int g=0; g < max; g++)
    {
        now = pow(10, g);
        now1 = now;            
        switch ((num / now1) % 10)    
        {    
            case 0:                    
                if (color==GREEN)       ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_0);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_0);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_0);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_0);
            break;
            case 1:
                 if (color==GREEN)      ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_1);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_1);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_1);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_1);
            break;
            case 2:
                 if (color==GREEN)      ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_2);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_2);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_2);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_2);
            break;
            case 3:
                if (color==GREEN)       ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_3);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_3);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_3);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_3);
            break;
            case 4:
                if (color==GREEN)       ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_4);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_4);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_4);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_4);
            break;
            case 5:
                if (color==GREEN)       ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_5);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_5);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_5);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_5);
            break;
            case 6:
                if (color==GREEN)       ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_6);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_6);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_6);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_6);
            break;
            case 7:
                if (color==GREEN)       ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_7);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_7);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_7);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_7);
            break;
            case 8:
                if (color==GREEN)       ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_8);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_8);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_8);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_8);
            break;
            case 9:
                if (color==GREEN)       ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)G_9);
                else if (color==RED)    ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)R_9);
                else if (color==YELLOW) ILI9341_DrawImage(X0 - BIG_NUM_W - g * BIG_NUM_W, Y0, BIG_NUM_W, BIG_NUM_H, (const uint16_t*)Y_9);
                else if (color==BLACK)  ILI9341_DrawImage(X0 - MID_NUM_W - g * MID_NUM_W, Y0, MID_NUM_W, MID_NUM_H, (const uint16_t*)B_L_9);
            break;
        }
    }
}


