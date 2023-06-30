#include "http_utils.h"
#include "jWrite.h"
#include "i2c.h"

char http_init[] = "AT+HTTPINIT";
char http_cid[] = "AT+HTTPPARA=CID,1";
char http_url[BUF_LEN] = "AT+HTTPPARA=\"URL\","; //"http://bp-technopark.mdapp.online/api/v3/users/login"
char http_json[] = "AT+HTTPPARA=\"CONTENT\",\"application/json\"";
char http_token[LONG_BUF_LEN] = "AT+HTTPPARA=\"USERDATA\",\"Authorization:Bearer \""; // здесь токен вроде бы
char http_len[] = "AT+HTTPDATA=61,20000"; //ƒлина строки, врем€ ожидани€
//{"app.username":"102@samsmu.ru","app.password":"aTXGZYqFhs"}
char http_post[] = "AT+HTTPACTION=1";
char http_read[] = "AT+HTTPREAD";
char http_term[] = "AT+HTTPTERM";

char url_buf[BUF_LEN];

char login_route[] = "/api/v3/users/login";
char measurement_route[] = "/api/v3/data/measurements?person_id=";

char token[500];
char patient_id[10];
char login_body[BUF_LEN];
char login[EEPROM_CELL_SIZE];
char pass[EEPROM_CELL_SIZE];
char measurement_body[LONG_BUF_LEN];

void ReadFromEEPROM()
{
    at24_HAL_ReadBytes(&hi2c1, EEPROM_BASE_ADDR, F_LOGIN * EEPROM_CELL_SIZE, login, EEPROM_CELL_SIZE);
    at24_HAL_ReadBytes(&hi2c1, EEPROM_BASE_ADDR, F_PASSWORD * EEPROM_CELL_SIZE, pass, EEPROM_CELL_SIZE);
    at24_HAL_ReadBytes(&hi2c1, EEPROM_BASE_ADDR, F_URL * EEPROM_CELL_SIZE, url_buf, EEPROM_CELL_SIZE);
}

void CreateLoginRequest()
{
    strcat(url_buf, login_route);
}

void CreateMeasurementRequest()
{
    strcat(url_buf, measurement_route);
    strcat(url_buf, patient_id);
}

int CreateLoginBody()
{
	jwOpen(login_body, BUF_LEN, JW_OBJECT, JW_PRETTY);
		jwObj_string("app.username", login);
		jwObj_string("app.password", pass);
	return jwClose();
}

int CreateMeasurementBody(int id, int sys, int dia, int pulse, RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate)
{
	int err;

	int string_buff[20] = { 0 };

	jwOpen(measurement_body, LONG_BUF_LEN, JW_ARRAY, JW_PRETTY);
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
	sprintf(string_buff, "%d-%02d-%02dT%02d:%02d:%02d", sDate.Year, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);
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
	jwObj_string("datetime", "2022-10-12T09:30:05");
	jwEnd();
	err = jwClose();
	//≈сли нужно добавить { в начало и } в конец. 
/*	ind = 0;
	while (measurement_body[ind] != 0) ind++;
	for (int i = 0; i <= ind; i++)
	{
		measurement_body[ind - i + 3] = measurement_body[ind - i];
	}
	measurement_body[0] = '{';
	measurement_body[1] = 0x0D;
	measurement_body[2] = 0x0A;
	measurement_body[ind + 3] = 0x0D;
	measurement_body[ind + 4] = 0x0A;
	measurement_body[ind + 5] = '}';
	measurement_body[ind + 6] = 0;*/
    return err;
}

uint16_t GetToken(char* json)
{
    return GetValueFromJSON(json, "security.access_token\": \"", token);
}

uint16_t GetPatientId(char* json)
{
    return GetValueFromJSON(json, "app.object.id\": \"", patient_id);
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
		if (index > 1024) return 0;
	}
	value[index] = 0;
	return index;
}

uint16_t FindIndex(char* buff, char* string_to_find)
{
	uint16_t ind = 0;
	uint16_t index = 0;
	uint16_t str_length;
	str_length = strlen(string_to_find);
	while (ind < 1024)
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
