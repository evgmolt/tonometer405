#include "http_utils.h"
#include "jWrite.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"

const char crlf[] = "\r"; //"\r\n"

const char gprs_cont[] = "AT+SAPBR=3,1,\"Contype\",\"GPRS\"";
const char gprs_apn[] = "AT+SAPBR=3,1,APN,";
const char gprs_connect[] = "AT+SAPBR=1,1"; //Установка соединения
const char gprs_getdata[] = "AT+SAPBR=2,1"; //Получение сведений, пока не используется, м.б. надо?
const char gprs_disconnect[] = "AT+SAPBR=0,1"; //Закрыть соединение

const char http_init[] = "AT+HTTPINIT";
const char http_cid[] = "AT+HTTPPARA=CID,1";
const char http_url[] = "AT+HTTPPARA=\"URL\","; 
const char http_content_plain[] = "AT+HTTPPARA=\"CONTENT\",\"text/plain\"";
const char http_content_json[] = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";
const char http_token[] = "AT+HTTPPARA=\"USERDATA\",\"Authorization:Bearer \""; // здесь токен вроде бы
const char http_len[] = "AT+HTTPDATA="; //Размер передаваемых данных (body), время ожидания
//Здесь передается тело запроса: {"app.username":"102@samsmu.ru","app.password":"aTXGZYqFhs"}
const char http_post[] = "AT+HTTPACTION=1"; //POST запрос
const char http_read[] = "AT+HTTPREAD"; //Читаем, что ответит сервер
const char http_term[] = "AT+HTTPTERM"; //Закрывает соединение

char common_buf[BUF_LEN]; //Буфер общего назначения

const char login_route[] = "/api/v3/users/login";
const char measurement_route[] = "/api/v3/data/measurements?person_id=";

char token[BUF_LEN]; 
char patient_id[6];
char login_body[BUF_LEN];

//Данные, считываемые из EEPROM
char e_login[EEPROM_CELL_SIZE];
char e_pass[EEPROM_CELL_SIZE];
char e_url[EEPROM_CELL_SIZE];
char e_point[EEPROM_CELL_SIZE];

uint8_t SendResultHttp(uint16_t p_sys, uint16_t p_dia, uint16_t pulse)
{
    if (InitGprs() != HAL_OK) return HAL_ERROR;
    if (SendLogin() != HAL_OK) return HAL_ERROR;
    uint16_t size;
	memset(common_buf, 0, LONG_BUF_LEN);
    if (HAL_UARTEx_ReceiveToIdle(&huart2, common_buf, LONG_BUF_LEN, size, RECEIVE_TIMEOUT) != HAL_OK) return HAL_ERROR; //Получаем ответ сервера
    if (GetValueFromJSON(common_buf, "security.access_token\": \"", token) == 0) return HAL_ERROR; //Извлекаем токен в буфер token
    if (GetValueFromJSON(common_buf, "app.object.id\": \"", patient_id) == 0) return HAL_ERROR; //Извлекаем ID в буфер patient_id
	memset(common_buf, 0, LONG_BUF_LEN);
	sprintf(common_buf, "%s\"%s%s%s\"", http_url, e_url, measurement_route, patient_id);
    if (SendDataWaitOk(common_buf) != HAL_OK) return HAL_ERROR;
    if (SendDataWaitOk(http_content_json) != HAL_OK) return HAL_ERROR;
	memset(common_buf, 0, LONG_BUF_LEN);
	sprintf(common_buf, "%s%s\"", http_token, token); //Нужны ли кавычки между Bearer и началом токена?
    if (SendDataWaitOk(common_buf) != HAL_OK) return HAL_ERROR;
    //Читаем текущее время и дату
    RTC_TimeTypeDef sTime = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    memset(common_buf, 0, LONG_BUF_LEN);
    CreateMeasurementBody(common_buf, atoi(patient_id), p_sys, p_dia, pulse, sTime, sDate);
    if (SendDataWaitOk(common_buf) != HAL_OK) return HAL_ERROR;
    if (SendDataWaitOk(http_post) != HAL_OK) return HAL_ERROR;
    if (SendDataWaitOk(http_term) != HAL_OK) return HAL_ERROR;  //закрываем http  
    if (SendDataWaitOk(gprs_disconnect) != HAL_OK) return HAL_ERROR;   //закрываем gprs  
    return HAL_OK;
}

uint8_t InitGprs()
{
    ReadFromEEPROM();
    if (SendDataWaitOk(gprs_cont) != HAL_OK) return HAL_ERROR;
	memset(common_buf, 0, LONG_BUF_LEN);
	sprintf(common_buf, "%s\"%s\"", gprs_apn, e_point);
    if (SendDataWaitOk(common_buf) != HAL_OK) return HAL_ERROR;
    if (SendDataWaitOk(gprs_connect) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

uint8_t SendLogin()
{
    if (SendDataWaitOk(http_init) != HAL_OK) return HAL_ERROR;
    if (SendDataWaitOk(http_cid) != HAL_OK) return HAL_ERROR;
	memset(common_buf, 0, LONG_BUF_LEN);
	sprintf(common_buf, "%s\"%s%s\"", http_url, e_url, login_route);
    if (SendDataWaitOk(common_buf) != HAL_OK) return HAL_ERROR;
    if (SendDataWaitOk(http_content_json) != HAL_OK) return HAL_ERROR;
	memset(login_body, 0, BUF_LEN);
    CreateLoginBody(login_body);
	memset(common_buf, 0, LONG_BUF_LEN);
    sprintf(common_buf, "%s%d%s", http_len, strlen(login_body), WAIT_TIME);
    if (SendDataWaitOk(common_buf) != HAL_OK) return HAL_ERROR;
    if (SendDataWaitOk(login_body) != HAL_OK) return HAL_ERROR;
    if (SendDataWaitOk(http_post) != HAL_OK) return HAL_ERROR;
    if (SendDataWaitOk(http_read) != HAL_OK) return HAL_ERROR;
}

void ReadFromEEPROM()
{
    at24_HAL_ReadBytes(&hi2c1, EEPROM_BASE_ADDR, F_LOGIN * EEPROM_CELL_SIZE, e_login, EEPROM_CELL_SIZE);
    at24_HAL_ReadBytes(&hi2c1, EEPROM_BASE_ADDR, F_PASSWORD * EEPROM_CELL_SIZE, e_pass, EEPROM_CELL_SIZE);
    at24_HAL_ReadBytes(&hi2c1, EEPROM_BASE_ADDR, F_URL * EEPROM_CELL_SIZE, e_url, EEPROM_CELL_SIZE);
    at24_HAL_ReadBytes(&hi2c1, EEPROM_BASE_ADDR, F_POINT * EEPROM_CELL_SIZE, e_point, EEPROM_CELL_SIZE);
}


int CreateLoginBody(char* buf)
{
	jwOpen(buf, BUF_LEN, JW_OBJECT, JW_PRETTY);
		jwObj_string("app.username", e_login);
		jwObj_string("app.password", e_pass);
	return jwClose();
}

int CreateMeasurementBody(char* buf, int id, int sys, int dia, int pulse, RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate)
{
	int err;

	int string_buff[20] = { 0 };

	jwOpen(buf, LONG_BUF_LEN, JW_ARRAY, JW_PRETTY);
	jwArr_object();
	sprintf(string_buff, "%d", id);
	jwObj_string("ownerId", string_buff);
	jwObj_string("source", "tonometer");
	jwObj_string("type", "med.blood_pressure");
	sprintf(string_buff, "%d/%d", sys, dia);
	jwObj_string("value", string_buff);
	jwObj_object("extraParams");
	jwObj_string("app.comment", "");
	jwEnd();
	sprintf(string_buff, "%d-%02d-%02dT%02d:%02d:%02d", 2000 + sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);
	jwObj_string("datetime", string_buff);
	jwEnd();

	jwArr_object();
	sprintf(string_buff, "%d", id);
	jwObj_string("ownerId", string_buff);
	jwObj_string("source", "tonometer");
	jwObj_string("type", "med.pulse");
	sprintf(string_buff, "%d", pulse);
	jwObj_string("value", string_buff);
	jwObj_object("extraParams");
	jwObj_string("app.comment", "");
	jwEnd();
	sprintf(string_buff, "%d-%02d-%02dT%02d:%02d:%02d", 2000 + sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);
	jwEnd();
	err = jwClose();
	//Если нужно добавить { в начало и } в конец. 
/*	ind = 0;
	while (buf[ind] != 0) ind++;
	for (int i = 0; i <= ind; i++)
	{
		measurement_body[ind - i + 3] = measurement_body[ind - i];
	}
	buf[0] = '{';
	buf[1] = 0x0D;
	buf[2] = 0x0A;
	buf[ind + 3] = 0x0D;
	buf[ind + 4] = 0x0A;
	buf[ind + 5] = '}';
	buf[ind + 6] = 0;*/
    return err;
}

uint16_t GetValueFromJSON(char* json, char* string_to_find, char* value)
{
	uint16_t pos;
	uint16_t index = 0;

	pos = FindIndex(json, string_to_find);
	if (pos == 0) return 0;
	pos++;
	while (json[pos] != '"')
	{
		value[index] = json[pos];
		pos++;
		index++;
		if (index > LONG_BUF_LEN) return 0;
	}
	value[index] = 0;
	return index;
}

//Возвращает индекс конца string_to_find в buff
uint16_t FindIndex(char* buff, char* string_to_find)
{
	uint16_t ind = 0;
	uint16_t index = 0;
	uint16_t str_length;
	str_length = strlen(string_to_find);
	while (ind < LONG_BUF_LEN)
	{
		if (buff[ind] == string_to_find[index])
		{
			index++;
			if (index == str_length) return ind;
		}
		else
		{
			index = 0;
		}
		ind++;
	}
	return 0;
}

uint8_t SendDataWaitOk(char* buff)
{
    strcat(buff, crlf); //Если нужно  Carriage Return Line Feed для SIM800 (проверить);
    HAL_UART_Transmit(&huart2, buff, strlen(buff), TRANSMIT_TIMEOUT);
    uint16_t size;
    uint8_t result = HAL_UARTEx_ReceiveToIdle(&huart2, common_buf, LONG_BUF_LEN, size, RECEIVE_TIMEOUT);
    if (FindIndex(buff, "OK") > 0) return result;
    return HAL_ERROR;
}
