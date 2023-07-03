#include "main.h"

#define BUF_LEN 0x200
#define LONG_BUF_LEN 0x600

#define WAIT_TIME "20000"
#define TRANSMIT_TIMEOUT 3000
#define RECEIVE_TIMEOUT 3000

const char http_init[];
const char http_cid[];
const char http_url[BUF_LEN];
const char http_json[];
const char http_token[LONG_BUF_LEN];
const char http_len[];
const char http_post[];
const char http_read[];
const char http_term[];

char common_buf[BUF_LEN];

const char login_route[];
const char measurement_route[];

extern char token[BUF_LEN];
extern char id[10];
extern char login_body[BUF_LEN];
extern char measurement_body[LONG_BUF_LEN];

extern char e_login[EEPROM_CELL_SIZE];
extern char e_pass[EEPROM_CELL_SIZE];
extern char e_url[EEPROM_CELL_SIZE];
extern char e_point[EEPROM_CELL_SIZE];

uint8_t SendResultHttp(uint16_t p_sys, uint16_t p_dya, uint16_t pulse);

uint8_t InitGprs();
uint8_t SendLogin();

void ReadFromEEPROM();
void CreateTokenString();
void CreateMeasurementRequest();
int CreateLoginBody(char* buf);
int CreateMeasurementBody(int id, int sys, int dia, int pulse, RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate);
uint16_t GetToken(char* json);
uint16_t GetPatientId(char* json);
uint16_t GetValueFromJSON(char* json, char* string_to_find, char* value);
uint16_t FindIndex(char* buff, char* string_to_find);

uint8_t SendDataWaitOk(char* buff);
    