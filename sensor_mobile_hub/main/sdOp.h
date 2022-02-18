 


#include "driver/gpio.h"

typedef struct 
{
    char *sData;
    uint8_t varId;
    char *codExp;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint16_t netId;
    uint8_t chipId;
} sdData; 

void setKey(uint16_t key);

void setId(uint8_t id);

void gravaLog();

void valoresTeste();

void startSd();

void gravaConfig();

void update();

void debugPrint();



