#include "sdOp.h"
#include <unistd.h>//TODO :checar includes desnecessarios
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <dirent.h>

#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
#endif
static const char *TAG = ".";

#define MOUNT_POINT "/sdcard"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define SPI_DMA_CHAN    host.slot

int countTask = 0;
int varId = 1;
char* dataLog = "teste";


sdData atual;
int count=0;

void startSd(){
    struct dirent *pDirent;
    DIR *pDir;
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SPI peripheral");
    ESP_LOGI(TAG,"%d",PIN_NUM_MOSI);
    ESP_LOGI(TAG,"%d",PIN_NUM_MISO);
    ESP_LOGI(TAG,"%d",PIN_NUM_CLK);
    ESP_LOGI(TAG,"%d",PIN_NUM_CS);
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    sdmmc_card_print_info(stdout, card);
    pDir = opendir (MOUNT_POINT);
        if (pDir == NULL) {
            ESP_LOGI(TAG, "Nao abri o diretorio");
        }else{
            ESP_LOGI(TAG, "Abri o diretorio");
        }
    while ((pDirent = readdir(pDir)) != NULL) {
        ESP_LOGI(TAG, "[%s]\n", pDirent->d_name);
        count++;     
    }
    ESP_LOGI(TAG,"O numero de arquivos e %d", count);

}

void valoresTeste(){
    atual.sData = malloc(16);
    strcpy(atual.sData,"Outro teste");
    atual.varId = 72;
    atual.codExp = malloc(16);
    strcpy(atual.codExp,"Exp1");
    atual.data1 = 43;
    atual.data2 = 2;
    atual.data3 = 3;
}

void gravaLog(){
    char* c1 = malloc(2);
    int cl2 = snprintf( NULL, 0, "%d", count );
    char* c2 = malloc( cl2 + 1 );
    snprintf( c2, cl2 + 1, "%d", count);
    char* c3 = malloc(2);
    int cl4 = snprintf( NULL, 0, "%d", atual.varId );
    char* c4 = malloc( cl4 + 1 );
    snprintf(c4,cl4 + 1,"%d",atual.varId);
    char *nomeArq=malloc(40);
    char* c7 = malloc(5);
    char* c5 = malloc(2);
    int cl6 = snprintf( NULL, 0, "%s", atual.codExp );
    ESP_LOGI(TAG,"LENGHT EXP %d", cl6);
    char* c6 = malloc( cl6 + 1 );
    snprintf(c6,cl6 + 1,"%s",atual.codExp);
    ESP_LOGI(TAG,"Print teste %s", c6);
    c1 = "/";
    c3 = "-";
    c5 = "-";
    c7 = ".txt";
    strcpy(nomeArq, MOUNT_POINT);
    strcat(nomeArq, c1);
    strcat(nomeArq, c2);
    strcat(nomeArq, c3);
    strcat(nomeArq, c4);
    strcat(nomeArq, c5);
    //strcat(nomeArq, c6);//comentado pois ao colocar a string --crash
    strcat(nomeArq, c7);
    ESP_LOGI(TAG,"Print teste %s", nomeArq);
    FILE* f = fopen(nomeArq, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
    }
    ESP_LOGI(TAG,"Print teste %s", atual.sData);
    ESP_LOGI(TAG,"Print teste %d", atual.data1);
    ESP_LOGI(TAG,"Print teste %d", atual.data2);
    ESP_LOGI(TAG,"Print teste %d", atual.data3);
    int cld1 = snprintf( NULL, 0, "%d", atual.data1 );
    char* d1 = malloc( cld1 + 1 );
    snprintf( d1, cld1 + 1, "%d", atual.data1 );
    int cld2 = snprintf( NULL, 0, "%d", atual.data2 );
    char* d2 = malloc( cld2 + 1 );
    snprintf( d2, cld2 + 1, "%d", atual.data2 );
    int cld3 = snprintf( NULL, 0, "%d", atual.data3 );
    char* d3 = malloc( cld3 + 1 );
    snprintf( d3, cld3 + 1, "%d", atual.data3 );
    fprintf(f,atual.sData);
    fprintf(f,"\n");
    fprintf(f,d1);
    fprintf(f,"\n");
    fprintf(f,d2);
    fprintf(f,"\n");
    fprintf(f,d3);
    fclose(f);   

} 


void setKey(uint16_t key){
    atual.netId = key;
}

void setId(uint8_t id){
    atual.chipId = id;
    ESP_LOGI(TAG,"---------------->chegou id %u\n",id);
}


void gravaConfig(){
    
    char *nomeArq=malloc(30);
    nomeArq = "/sdcard/config.txt";
    FILE* f = fopen(nomeArq, "w");
    int cld1 = snprintf( NULL, 0, "%d", atual.data1 );
    char* d1 = malloc( cld1 + 1 );
    snprintf( d1, cld1 + 1, "%d", atual.data1 );
    int cld2 = snprintf( NULL, 0, "%d", atual.data2 );
    char* d2 = malloc( cld2 + 1 );
    snprintf( d2, cld2 + 1, "%d", atual.data2 );
    int cld3 = snprintf( NULL, 0, "%d", atual.data3 );
    char* d3 = malloc( cld3 + 1 );
    snprintf( d3, cld3 + 1, "%d", atual.data3 );
    int idl4 = snprintf( NULL, 0, "%d", atual.varId);
    char* id = malloc( idl4 + 1 );
    snprintf(id,idl4 + 1,"%d",atual.varId);
    int netl = snprintf( NULL, 0, "%d", atual.netId);
    char* net = malloc( netl + 1 );
    snprintf(net,netl + 1,"%d",atual.netId);
    int chipl = snprintf( NULL, 0, "%d", atual.chipId);
    char* chip = malloc( chipl + 1 );
    snprintf(chip,chipl + 1,"%d",atual.chipId);
    ESP_LOGI(TAG, "dentro do gravaConfig");
    ESP_LOGI(TAG, "d3-->%u", atual.data3);
    ESP_LOGI(TAG, "codExp-->%s", atual.codExp);
    ESP_LOGI(TAG, "netId-->%u", atual.netId);
    fprintf(f,atual.sData);
    fprintf(f,"\n");
    fprintf(f,atual.codExp);
    fprintf(f,"\n");
    fprintf(f,d1);
    fprintf(f,"\n");
    fprintf(f,d2);
    fprintf(f,"\n");
    fprintf(f,d3);
    fprintf(f,"\n");
    fprintf(f,id);
    fprintf(f,"\n");
    fprintf(f,net);
    fprintf(f,"\n");
    fprintf(f,chip);
    fclose(f);   


}




void update(){
  
    char sData[16];
    char codExp[16];
    uint8_t d1=0;
    uint8_t d2=0;
    uint8_t d3=0;
    uint8_t varId=0;
    int count = 0;
    ESP_LOGI(TAG, "dentro do update");
    char * buffer = 0;
    long length;
    FILE *file;
    file = fopen("/sdcard/config.txt", "rb");
    if (file)
    {
        fseek (file, 0, SEEK_END);
        length = ftell (file);
        ESP_LOGI(TAG, "length-->%ld", length);
        fseek (file, 0, SEEK_SET);
        buffer = malloc (length);
        if (buffer)
        {
            fread (buffer, 1, length, file);
        }
        fclose (file);
    }
    ESP_LOGI(TAG, "buffer-->%s", buffer);
    char* linha = malloc(16);
    int i=0;
    
    if (buffer)
    {
        
        char *ptr;
        ptr = strtok(buffer,"\n");
	    while(ptr != NULL)
	    {
            
            switch(i)
            {   
                
                case(0):
                    ESP_LOGI(TAG,"ptr  ->%s", ptr);
                    ESP_LOGI(TAG,"len  ->%d", strlen(ptr));
                    memmove(atual.sData,ptr,strlen(ptr));
                    ESP_LOGI(TAG,"sData  ->%s", atual.sData);
                    break;
                  
                case(1):
                    ESP_LOGI(TAG,"ptr  ->%s", ptr);
                    ESP_LOGI(TAG,"len  ->%d", strlen(ptr));
                    memmove(atual.codExp,ptr,strlen(ptr));
                    ESP_LOGI(TAG,"codExp  ->%s", atual.codExp);
                    break;
                
                case(2):
                    ESP_LOGI(TAG,"ptr  ->%s", ptr);
                    atual.data1=atoi(ptr);
                    ESP_LOGI(TAG,"data1  ->%d", atual.data1);
                    break;
                   
                case(3):
                    atual.data2=atoi(ptr);
                    break;
                   
                case(4):
                    atual.data3=atoi(ptr);
                    break;
                    
                case(5):
                    atual.varId=atoi(ptr);
                    break;
                    
                case(6):
                    atual.netId=atoi(ptr);
                    break;
                   
                case(7):
                    atual.chipId=atoi(ptr);
                    ESP_LOGI(TAG,"chipId  ->%d", atual.chipId);
                    break;
                    
            }
            i++;
		    ptr = strtok(NULL, "\n");
	    }
    }   
}                                                                           


/* task que havia criado para fazer log periodicamente
void taskSd(void *params){
    while(true){
        char* c1 = malloc(2);
        int atual = countTask +(int)params;
        ESP_LOGI(TAG,"Contador da task %d", atual);
        countTask++;
        int cl2 = snprintf( NULL, 0, "%d", atual );
        char* c2 = malloc( cl2 + 1 );
        snprintf( c2, cl2 + 1, "%d", atual );
        char* c3 = malloc(2);
        int cl4 = snprintf( NULL, 0, "%d", varId );
        char* c4 = malloc( cl4 + 1 );
        snprintf( c4, cl4 + 1, "%d", varId );
        char *nomeArq=malloc( 30 );
        strcpy(nomeArq, c1);
        strcat(nomeArq, c2);
        char* c5 = malloc(5);
        c1 = "/";
        c3 = "-";
        c5 = ".txt";
        strcpy(nomeArq, MOUNT_POINT);
        strcat(nomeArq, c1);
        strcat(nomeArq, c2);
        strcat(nomeArq, c3);
        strcat(nomeArq, c4);
        strcat(nomeArq, c5);
        ESP_LOGI(TAG,"Print teste %s", nomeArq);
        FILE* f = fopen(nomeArq, "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
        }
        fprintf(f,dataLog);
        fclose(f);   
        vTaskDelay(5000/portTICK_PERIOD_MS);
    }
}
*/

void debugPrint(){
    ESP_LOGI(TAG, "netKey---->%u", atual.netId);
}

