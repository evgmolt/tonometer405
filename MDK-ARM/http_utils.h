#include "main.h"

#define BUF_LEN 0x200
#define LONG_BUF_LEN 0x600

//Настройка HTTP ------------
char http_init[];
char http_cid[];
char http_url[BUF_LEN];
char http_json[];
char http_token[LONG_BUF_LEN];
char http_len[];
char http_post[];
char http_read[];
char http_term[];

char url_buf[BUF_LEN];

char login_route[];
char measurement_route[];

extern char token[500];
extern char id[10];
extern char login_body[BUF_LEN];
extern char measurement_body[LONG_BUF_LEN];

void ReadFromEEPROM();
void CreateLoginRequest();
void CreateMeasurementRequest();

int CreateLoginBody();
int CreateMeasurementBody(int id, int sys, int dia, int pulse, RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate);
uint16_t GetToken(char* json);
uint16_t GetPatientId(char* json);
uint16_t GetValueFromJSON(char* json, char* string_to_find, char* value);
uint16_t FindIndex(char* buff, char* string_to_find);