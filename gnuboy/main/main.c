/**
 * Copyright (c) 2022 Nicolai Electronics
 * Copyright (c) 2019 Fuji Pebri
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <string.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_err.h>
#include <esp_log.h>
#include "hardware.h"

#include <loader.h>
#include <hw.h>
#include <lcd.h>
#include <fb.h>
#include <cpu.h>
#include <mem.h>
#include <sound.h>
#include <pcm.h>
#include <regs.h>
#include <rtc_gnuboy.h>
#include <gnuboy.h>
#include <audio.h>

#include <sdcard.h>
#include "sd_fail.h"

static const char *TAG = "main";
extern void cpu_reset();
extern int cpu_emulate(int cycles);

extern void loadstate_rom(const unsigned char* rom);

struct fb fb;
struct pcm pcm;

#define SD_BASE_PATH "/sd"

#define GAMEBOY_WIDTH (160)
#define GAMEBOY_HEIGHT (144)

#define AUDIO_SAMPLE_RATE (32000)

uint16_t* displayBuffer[2]; //= { fb0, fb0 }; //[160 * 144];
uint8_t currentBuffer = 0;

int32_t* audioBuffer[2];
volatile uint8_t currentAudioBuffer = 0;
volatile uint16_t currentAudioSampleCount;
volatile int16_t* currentAudioBufferPtr;

int frame = 0;
uint elapsedTime = 0;

QueueHandle_t vidQueue;
QueueHandle_t audioQueue;

bool calibrate = true;
bool display_bno_value = true;
ILI9341* ili9341 = NULL;
uint16_t* framebuffer = NULL;

void LoadState();
void SaveState();
void LoadSram();
void SaveSram();


char* get_rom_name_settings() {
    return "lsdj.gb";
}

void button_handler(uint8_t pin, bool value) {
    switch(pin) {
        case PCA9555_PIN_BTN_START:
            pad_set(PAD_START, value);
            break;
        case PCA9555_PIN_BTN_SELECT:
            pad_set(PAD_SELECT, value);
            break;
        case PCA9555_PIN_BTN_MENU:
            if (value) LoadSram();
            break;
        case PCA9555_PIN_BTN_HOME:
            if (value) SaveSram();//SaveState();
            break;
        case PCA9555_PIN_BTN_JOY_LEFT:
            pad_set(PAD_LEFT, value);
            break;
        case PCA9555_PIN_BTN_JOY_PRESS:
            // Not used
            break;
        case PCA9555_PIN_BTN_JOY_DOWN:
            pad_set(PAD_DOWN, value);
            break;
        case PCA9555_PIN_BTN_JOY_UP:
            pad_set(PAD_UP, value);
            break;
        case PCA9555_PIN_BTN_JOY_RIGHT:
            pad_set(PAD_RIGHT, value);
            break;
        case PCA9555_PIN_BTN_BACK:
            pad_set(PAD_B, value);
            break;
        case PCA9555_PIN_BTN_ACCEPT:
            pad_set(PAD_A, value);
            break;
        default:
            // Unknown button, ignored
            break;
    }
}

void button_init() {
    PCA9555* pca9555 = get_pca9555();
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_START, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_SELECT, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_MENU, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_HOME, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_JOY_LEFT, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_JOY_PRESS, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_JOY_DOWN, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_JOY_UP, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_JOY_RIGHT, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_BACK, true);
    pca9555_set_gpio_polarity(pca9555, PCA9555_PIN_BTN_ACCEPT, true);
    
    pca9555->pin_state = 0; // Reset all pin states so that the interrupt function doesn't trigger all the handlers because we inverted the polarity :D
    
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_START, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_SELECT, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_MENU, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_HOME, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_JOY_LEFT, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_JOY_PRESS, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_JOY_DOWN, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_JOY_UP, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_JOY_RIGHT, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_BACK, button_handler);
    pca9555_set_interrupt_handler(pca9555, PCA9555_PIN_BTN_ACCEPT, button_handler);
}

void restart() {
    printf("Restarting...\n");
    fflush(stdout);
    esp_restart();
}

#define AVERAGE(a, b) ( ((((a) ^ (b)) & 0xf7deU) >> 1) + ((a) & (b)) )

#define LINE_BUFFERS (7)
#define LINE_COUNT   (19)
#define LINE_COUNT_UNSCALED   (4)

uint16_t* line[LINE_BUFFERS];

void clear_display() {
    int calc_line = 0;
    for (int y=0; y<ILI9341_HEIGHT; y++) {
        for (int x=0; x<ILI9341_WIDTH; x++) {
            line[calc_line][x] = 0;
        }
        ili9341_write_partial_direct(ili9341, (uint8_t*) line[calc_line], 0, y, ILI9341_WIDTH, 1);
        calc_line=(calc_line==1)?0:1;
    }
}

static uint16_t getPixel(const uint16_t * bufs, int x, int y, int w1, int h1, int w2, int h2)
{
    int x_diff, y_diff, xv, yv, red , green, blue, col, a, b, c, d, index;
    int x_ratio = (int) (((w1-1)<<16)/w2) + 1;
    int y_ratio = (int) (((h1-1)<<16)/h2) + 1;

    xv = (int) ((x_ratio * x)>>16);
    yv = (int) ((y_ratio * y)>>16);

    x_diff = ((x_ratio * x)>>16) - (xv);
    y_diff = ((y_ratio * y)>>16) - (yv);

    index = yv*w1+xv ;

    a = bufs[index];
    b = bufs[index+1];
    c = bufs[index+w1];
    d = bufs[index+w1+1];

    red = (((a >> 11) & 0x1f) * (1-x_diff) * (1-y_diff) + ((b >> 11) & 0x1f) * (x_diff) * (1-y_diff) +
        ((c >> 11) & 0x1f) * (y_diff) * (1-x_diff) + ((d >> 11) & 0x1f) * (x_diff * y_diff));

    green = (((a >> 5) & 0x3f) * (1-x_diff) * (1-y_diff) + ((b >> 5) & 0x3f) * (x_diff) * (1-y_diff) +
        ((c >> 5) & 0x3f) * (y_diff) * (1-x_diff) + ((d >> 5) & 0x3f) * (x_diff * y_diff));

    blue = (((a) & 0x1f) * (1-x_diff) * (1-y_diff) + ((b) & 0x1f) * (x_diff) * (1-y_diff) +
        ((c) & 0x1f) * (y_diff) * (1-x_diff) + ((d) & 0x1f) * (x_diff * y_diff));

    col = ((int)red << 11) | ((int)green << 5) | ((int)blue);

    return col;
}

void write_gb_frame(const uint16_t * data)
{
    short x,y;
    int sending_line=-1;
    int calc_line=0;
    
    if (data == NULL) return;
    
    bool scaling = true;
    
    if (scaling) {            
        int outputHeight = ILI9341_HEIGHT - 50;
        int outputWidth = GAMEBOY_WIDTH + (ILI9341_HEIGHT - 50 - GAMEBOY_HEIGHT);
        int xpos = (ILI9341_WIDTH - outputWidth) / 2;
        for (y=0; y<outputHeight; y+=LINE_COUNT)  {
            for (int i = 0; i < LINE_COUNT; ++i)
            {
                if((y + i) >= outputHeight) break;

                int index = (i) * outputWidth;
            
                for (x=0; x<outputWidth; ++x) 
                {
                    
                    uint16_t sample = getPixel(data, x, (y+i), GAMEBOY_WIDTH, GAMEBOY_HEIGHT, outputWidth, outputHeight);
                    line[calc_line][index]=((sample >> 8) | ((sample) << 8));
                    index++;
                }
            }
            sending_line=calc_line;
            calc_line = (calc_line + 1) % LINE_BUFFERS;
            ili9341_write_partial_direct(ili9341, (uint8_t*) line[sending_line], xpos, y + 25, outputWidth, LINE_COUNT);
        }
    }
    else
    {
        int ypos = (ILI9341_HEIGHT - GAMEBOY_HEIGHT)/2;
        int xpos = (ILI9341_WIDTH - GAMEBOY_WIDTH)/2;

        for (y=0; y<GAMEBOY_HEIGHT; y+=LINE_COUNT_UNSCALED)
        {
            for (int i = 0; i < LINE_COUNT_UNSCALED; ++i)
            {
                if((y + i) >= GAMEBOY_HEIGHT) break;

                int index = (i) * GAMEBOY_WIDTH;
                int bufferIndex = ((y + i) * GAMEBOY_WIDTH);

                for (x = 0; x < GAMEBOY_WIDTH; ++x)
                {
                    uint16_t sample = data[bufferIndex++];
                    line[calc_line][index++] = ((sample >> 8) | ((sample & 0xff) << 8));
                }
            }
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            ili9341_write_partial_direct(ili9341, (uint8_t*) line[sending_line], xpos, y+ypos, GAMEBOY_WIDTH, LINE_COUNT_UNSCALED);
        }
    }
}

volatile bool videoTaskIsRunning = false;
void videoTask(void *arg) {
  videoTaskIsRunning = true;
  uint16_t* param;
  while(1) {
        xQueuePeek(vidQueue, &param, portMAX_DELAY);
        if (param == 1) break;
        write_gb_frame(param);
        xQueueReceive(vidQueue, &param, portMAX_DELAY);
    }
    videoTaskIsRunning = false;
    vTaskDelete(NULL);
}

void run_to_vblank() {
  /* FRAME BEGIN */

  /* FIXME: djudging by the time specified this was intended
  to emulate through vblank phase which is handled at the
  end of the loop. */
  cpu_emulate(2280);

  /* FIXME: R_LY >= 0; comparsion to zero can also be removed
  altogether, R_LY is always 0 at this point */
  while (R_LY > 0 && R_LY < 144)
  {
    /* Step through visible line scanning phase */
    emu_step();
  }

  /* VBLANK BEGIN */

  if ((frame % 2) == 0)
  {
      xQueueSend(vidQueue, &framebuffer, portMAX_DELAY);

      // swap buffers
      currentBuffer = currentBuffer ? 0 : 1;
      framebuffer = displayBuffer[currentBuffer];

      fb.ptr = framebuffer;
  }
  rtc_tick();

  sound_mix();

  if (pcm.pos > 100)
  {
        currentAudioBufferPtr = (int16_t*) audioBuffer[currentAudioBuffer];
        currentAudioSampleCount = pcm.pos;

        void* tempPtr = (void*) 0x1234;
        xQueueSend(audioQueue, &tempPtr, portMAX_DELAY);

        // Swap buffers
        currentAudioBuffer = currentAudioBuffer ? 0 : 1;
        pcm.buf = audioBuffer[currentAudioBuffer];
        pcm.pos = 0;
  }

  if (!(R_LCDC & 0x80)) {
    /* LCDC operation stopped */
    /* FIXME: djudging by the time specified, this is
    intended to emulate through visible line scanning
    phase, even though we are already at vblank here */
    cpu_emulate(32832);
  }

  while (R_LY > 0) {
    /* Step through vblank phase */
    emu_step();
  }
}

char* system_util_GetFileName(const char* path)
{
    int length = strlen(path);
    int fileNameStart = length;

    if (fileNameStart < 1) abort();

    while (fileNameStart > 0)
    {
        if (path[fileNameStart] == '/')
        {
            ++fileNameStart;
            break;
        }

        --fileNameStart;
    }

    int size = length - fileNameStart + 1;

    char* result = malloc(size);
    if (!result) abort();

    result[size - 1] = 0;
    for (int i = 0; i < size - 1; ++i)
    {
        result[i] = path[fileNameStart + i];
    }

    printf("GetFileName: result='%s'\n", result);

    return result;
}

char* sdcard_create_savefile_path(const char* base_path, const char* fileName, const char* ext)
{
    char* result = NULL;

    if (!base_path) abort();
    if (!fileName) abort();

    printf("%s: base_path='%s', fileName='%s'\n", __func__, base_path, fileName);

    // Determine folder
    char* extension = fileName + strlen(fileName); // place at NULL terminator
    while (extension != fileName)
    {
        if (*extension == '.')
        {
            ++extension;
            break;
        }
        --extension;
    }

    if (extension == fileName)
    {
        printf("%s: File extention not found.\n", __func__);
        abort();
    }

    //printf("%s: extension='%s'\n", __func__, extension);

    const char* DATA_PATH = "/gnuboy/";

    size_t savePathLength = strlen(base_path) + strlen(DATA_PATH) + strlen(fileName) + strlen(ext) + 1;
    char* savePath = malloc(savePathLength);
    if (savePath)
    {
        strcpy(savePath, base_path);
        strcat(savePath, DATA_PATH);
        strcat(savePath, fileName);
        strcat(savePath, ext);

        printf("%s: savefile_path='%s'\n", __func__, savePath);

        result = savePath;
    }

    return result;
}

static bool isOpen = false;

size_t sdcard_copy_file_to_memory(const char* path, void* ptr)
{
    size_t ret = 0;

    if (!isOpen)
    {
        printf("sdcard_copy_file_to_memory: not open.\n");
    }
    else
    {
        if (!ptr)
        {
            printf("sdcard_copy_file_to_memory: ptr is null.\n");
        }
        else
        {
            FILE* f = fopen(path, "rb");
            if (f == NULL)
            {
                printf("sdcard_copy_file_to_memory: fopen failed.\n");
            }
            else
            {
                // copy
                const size_t BLOCK_SIZE = 512;
                while(true)
                {
                    __asm__("memw");
                    size_t count = fread((uint8_t*)ptr + ret, 1, BLOCK_SIZE, f);
                    __asm__("memw");

                    ret += count;

                    if (count < BLOCK_SIZE) break;
                }
            }
        }
    }

    return ret;
}

void LoadState() {
    char* romName = get_rom_name_settings();
    if (romName == NULL) return;
    char* fileName = system_util_GetFileName(romName);
    if (!fileName) abort();
    char* pathName = sdcard_create_savefile_path(SD_BASE_PATH, fileName, ".sta");
    if (!pathName) abort();
    FILE* f = fopen(pathName, "r");
    if (f == NULL) {
        printf("LoadState: fopen load failed\n");
    } else {
        loadstate(f);
        fclose(f);
        vram_dirty();
        pal_dirty();
        sound_dirty();
        mem_updatemap();
        printf("LoadState: loadstate OK.\n");
    }
    free(pathName);
    free(fileName);
}

void SaveState() {
    char* romPath = get_rom_name_settings();
    if (romPath == NULL) return;
    char* fileName = system_util_GetFileName(romPath);
    if (!fileName) abort();
    char* pathName = sdcard_create_savefile_path(SD_BASE_PATH, fileName, ".sta");
    if (!pathName) abort();
    FILE* f = fopen(pathName, "w");
    if (f == NULL) {
        printf("%s: fopen save failed (%s)\n", __func__, pathName);
        free(pathName);
        free(fileName);
        return;
    }
    savestate(f);
    fclose(f);
    printf("%s: savestate OK.\n", __func__);
    free(pathName);
    free(fileName);
}

void LoadSram() {
    char* romName = get_rom_name_settings();
    if (romName == NULL) return;
    char* fileName = system_util_GetFileName(romName);
    if (!fileName) abort();
    char* pathName = sdcard_create_savefile_path(SD_BASE_PATH, fileName, ".srm");
    if (!pathName) abort();
    FILE* f = fopen(pathName, "r");
    if (f == NULL) {
        printf("SRAM load failed\n");
    } else {
        sram_load(f);
        fclose(f);
        vram_dirty();
        pal_dirty();
        sound_dirty();
        mem_updatemap();
        printf("SRAM loaded.\n");
    }
    free(pathName);
    free(fileName);
}

void SaveSram() {
    char* romPath = get_rom_name_settings();
    if (romPath == NULL) return;
    char* fileName = system_util_GetFileName(romPath);
    if (!fileName) abort();
    char* pathName = sdcard_create_savefile_path(SD_BASE_PATH, fileName, ".srm");
    if (!pathName) abort();
    FILE* f = fopen(pathName, "w");
    if (f == NULL) {
        printf("SRAM save failed\n");
        free(pathName);
        free(fileName);
        return;
    }
    sram_save(f);
    fclose(f);
    printf("SRAM saved.");
    free(pathName);
    free(fileName);
}


int pcm_submit()
{
    audio_submit(currentAudioBufferPtr, currentAudioSampleCount >> 1);

    return 1;
}

volatile bool AudioTaskIsRunning = false;
void audioTask(void* arg)
{
  // sound
  uint16_t* param;

  AudioTaskIsRunning = true;
  while(1)
  {
    xQueuePeek(audioQueue, &param, portMAX_DELAY);

    if (param == 0)
    {
        // TODO: determine if this is still needed
        abort();
    }
    else if (param == 1)
    {
        break;
    }
    else
    {
        pcm_submit();
    }

    xQueueReceive(audioQueue, &param, portMAX_DELAY);
  }

  printf("audioTask: exiting.\n");

  AudioTaskIsRunning = false;
  vTaskDelete(NULL);

  while (1) {}
}

uint8_t* load_file_to_ram(FILE* fd, size_t* fsize) {
    fseek(fd, 0, SEEK_END);
    *fsize = ftell(fd);
    fseek(fd, 0, SEEK_SET);
    uint8_t* file = heap_caps_malloc(*fsize, MALLOC_CAP_SPIRAM);
    if (file == NULL) return NULL;
    fread(file, *fsize, 1, fd);
    return file;
}

void app_main(void) {
    esp_err_t res;
    
    displayBuffer[0] = heap_caps_malloc(160 * 144 * 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    displayBuffer[1] = heap_caps_malloc(160 * 144 * 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    framebuffer = displayBuffer[0];
    currentBuffer = 0;
    
    if (displayBuffer[0] == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fb0!");
        restart();
    }

    if (displayBuffer[1] == NULL) {
        ESP_LOGE(TAG, "Failed to allocate fb1!");
        restart();
    }
    
    for (int i = 0; i < 2; ++i){
        memset(displayBuffer[i], 0, 160 * 144 * 2);
    }
    
    printf("app_main: displayBuffer[0]=%p, [1]=%p\n", displayBuffer[0], displayBuffer[1]);
    
    const size_t lineSize = ILI9341_WIDTH * LINE_COUNT * sizeof(uint16_t);
    for (int x = 0; x < LINE_BUFFERS; x++)
    {
        line[x] = heap_caps_malloc(lineSize, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (line[x] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate line %u!", x);
            restart();
        }
        memset(line[x], 0, lineSize);
    }
    
    res = hardware_init();
    if (res != ESP_OK) {
        printf("Failed to initialize hardware!\n");
        restart();
    }
    
    ili9341 = get_ili9341();
    
    audio_init(AUDIO_SAMPLE_RATE);
    
    /*framebuffer = heap_caps_malloc(ILI9341_BUFFER_SIZE, MALLOC_CAP_8BIT);
    if (framebuffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer");
        restart();
    }*/
    
    /*memset(framebuffer, 0, ILI9341_BUFFER_SIZE); // Clear framebuffer
    res = ili9341_write(ili9341, framebuffer);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write framebuffer to LCD");
        restart();
    }*/
    
    button_init();
    
    res = mount_sd(SD_CMD, SD_CLK, SD_D0, SD_PWR, SD_BASE_PATH, false, 5);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card, halted.");
        res = ili9341_write(ili9341, sd_fail);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write SD failure screen to LCD");
        }
        restart();
    }

    char* filename = get_rom_name_settings();
    const char* DATA_PATH = "/gnuboy/";
    size_t romPathLength = strlen(SD_BASE_PATH) + strlen(DATA_PATH) + strlen(filename) + 1;
    char* romPath = malloc(romPathLength);
    romPath[0] = 0;
    strcat(romPath, SD_BASE_PATH);
    strcat(romPath, DATA_PATH);
    strcat(romPath, filename);
    
    FILE* rom_fd = fopen(romPath, "rb");
    if (rom_fd == NULL) {
        ESP_LOGE(TAG, "Failed to open %s from the SD card", romPath);
        res = ili9341_write(ili9341, sd_fail);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write SD failure screen to LCD");
        }
        restart();
    }
    
    free(romPath);
    
    size_t rom_length;
    uint8_t* rom = load_file_to_ram(rom_fd, &rom_length);
    fclose(rom_fd);
    
    loader_init(rom);
    
    vidQueue = xQueueCreate(1, sizeof(uint16_t*));
    xTaskCreatePinnedToCore(&videoTask, "videoTask", 4096, NULL, 5, NULL, 1);
    audioQueue = xQueueCreate(1, sizeof(uint16_t*));
    xTaskCreatePinnedToCore(&audioTask, "audioTask", 2048, NULL, 5, NULL, 1); //768
        
    // Note: Magic number obtained by adjusting until audio buffer overflows stop.
    const int audioBufferLength = AUDIO_SAMPLE_RATE / 10 + 1;
    //printf("CHECKPOINT AUDIO: HEAP:0x%x - allocating 0x%x\n", esp_get_free_heap_size(), audioBufferLength * sizeof(int16_t) * 2 * 2);
    const int AUDIO_BUFFER_SIZE = audioBufferLength * sizeof(int16_t) * 2;
    
    emu_reset();
    
    //&rtc.carry, &rtc.stop,
    rtc.d = 1;
    rtc.h = 1;
    rtc.m = 1;
    rtc.s = 1;
    rtc.t = 1;

    // vid_begin
    memset(&fb, 0, sizeof(fb));
    fb.w = 160;
    fb.h = 144;
    fb.pelsize = 2;
    fb.pitch = fb.w * fb.pelsize;
    fb.indexed = 0;
    fb.ptr = framebuffer;
    fb.enabled = 1;
    fb.dirty = 0;
    
    // pcm.len = count of 16bit samples (x2 for stereo)
    memset(&pcm, 0, sizeof(pcm));
    pcm.hz = AUDIO_SAMPLE_RATE;
  	pcm.stereo = 1;
  	pcm.len = /*pcm.hz / 2*/ audioBufferLength;
  	pcm.buf = (int16_t*) heap_caps_malloc(AUDIO_BUFFER_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  	pcm.pos = 0;

    audioBuffer[0] = (int32_t*) pcm.buf;
    audioBuffer[1] = (int32_t*) heap_caps_malloc(AUDIO_BUFFER_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    
    if (audioBuffer[0] == NULL) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer 0");
        restart();
    }
    
    if (audioBuffer[1] == NULL) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer 1");
        restart();
    }
    
    lcd_begin();
    
    //clear_display();
    
    //LoadState();
    
    LoadSram();
    
    sound_reset();
    
    uint startTime;
    uint stopTime;
    uint totalElapsedTime = 0;
    uint actualFrameCount = 0;
    
    while (true) {
        startTime = xthal_get_ccount();
        run_to_vblank();
        stopTime = xthal_get_ccount();

        if (stopTime > startTime)
          elapsedTime = (stopTime - startTime);
        else
          elapsedTime = ((uint64_t)stopTime + (uint64_t)0xffffffff) - (startTime);

        totalElapsedTime += elapsedTime;
        ++frame;
        ++actualFrameCount;

        if (actualFrameCount == 60)
        {
          float seconds = totalElapsedTime / (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000.0f); // 240000000.0f; // (240Mhz)
          float fps = actualFrameCount / seconds;

          printf("HEAP:0x%x, FPS:%f\n", esp_get_free_heap_size(), fps);

          actualFrameCount = 0;
          totalElapsedTime = 0;
        }
    }
}
