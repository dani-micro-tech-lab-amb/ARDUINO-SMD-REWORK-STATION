#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    pin_t brk_pin;
    pin_t    cs_pin;
    uint32_t spi;
    uint8_t  spi_buffer[2];
    uint32_t attr_thTemp;
} chip_state_t;

static void fillData(chip_state_t *chip) {
    float t = attr_read_float(chip->attr_thTemp);
    uint16_t d = t * (1 / 0.25);
    bool isOpen = pin_read(chip->brk_pin); 
    chip->spi_buffer[1] = ((d & 0xff) << 3) | (isOpen << 2);
    chip->spi_buffer[0] = (d >> 5);
}

void cs_change(void *u_data, pin_t pin, uint32_t bv) {
    chip_state_t *chip = (chip_state_t*)u_data;
    if (bv) {
        
        spi_stop(chip->spi);
    } else {
        fillData(chip);
        spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
        //printf("cs low\n");
    }
}

void spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *chip = (chip_state_t*)user_data;
  //printf("spi done\n");
  //printf("count: %u\n", count);
  // nulla da fare
  //spi_stop(chip->spi);
}

void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  chip->cs_pin = pin_init("CS", INPUT_PULLUP);
  
  
  //tempCelsius
  //thTemp
  uint32_t attr_dia_temp = attr_init("tempCelsius", 25);
  chip->attr_thTemp = attr_init("thTemp", attr_read(attr_dia_temp));
  
  fillData(chip);

  const pin_watch_config_t watch_cfg = {
    .edge = BOTH,
    .pin_change = cs_change,
    .user_data = chip,
  };
  pin_watch(chip->cs_pin, &watch_cfg);

  const spi_config_t spi_cfg = {
    .sck = pin_init("SCK", INPUT),
    .miso = pin_init("SO", INPUT),
    .mosi = NO_PIN,
    .mode = 0,
    .done = spi_done,
    .user_data = chip
  };
  chip->spi = spi_init(&spi_cfg);

  // TODO: Initialize the chip, set up IO pins, create timers, etc.

  //printf("Hello from custom chip!\n");
}

