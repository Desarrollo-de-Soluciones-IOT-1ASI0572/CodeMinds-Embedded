#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RFID_UID_COUNT 5
#define RFID_UID_LENGTH 4

const uint8_t rfid_uids[RFID_UID_COUNT][RFID_UID_LENGTH] = {
    {0x12, 0x34, 0x56, 0x78},
    {0xAB, 0xCD, 0xEF, 0x01},
    {0x23, 0x45, 0x67, 0x89},
    {0x9A, 0xBC, 0xDE, 0xF0},
    {0x11, 0x22, 0x33, 0x44}};

typedef struct
{
  uint32_t uid_index;
  pin_t miso_pin; // Almacena el identificador del pin MISO
  pin_t sck_pin;  // Pin SCK para detectar comunicación SPI
  pin_t mosi_pin; // Pin MOSI para recibir comandos
  pin_t sda_pin;  // Pin SDA para selección de chip
} chip_state_t;

static void chip_timer_event(void *user_data);

void chip_init()
{
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  // Setup SPI pins
  chip->mosi_pin = pin_init("MOSI", INPUT);
  chip->miso_pin = pin_init("MISO", OUTPUT); // Inicializa el pin MISO y almacena su identificador
  chip->sck_pin = pin_init("SCK", INPUT);
  chip->sda_pin = pin_init("SDA", INPUT_PULLUP);

  // Setup timer to simulate RFID detection
  const timer_config_t timer_config = {
      .callback = chip_timer_event,
      .user_data = chip};
  timer_t timer = timer_init(&timer_config);
  timer_start(timer, 2000000, true); // Trigger every 2 seconds

  chip->uid_index = 0;
  printf("RFID simulation started.\n");
}

static void chip_timer_event(void *user_data)
{
  chip_state_t *chip = (chip_state_t *)user_data;

  // Simulate SPI communication: Check if SDA is active (chip selected)
  if (pin_read(chip->sda_pin) == 0)
  { // SDA active low
    printf("SPI communication detected.\n");

    // Read command from MOSI
    uint8_t command = pin_read(chip->mosi_pin);
    printf("Received command: %02X\n", command);

    // Respond with UID if the command matches
    if (command == 0x30)
    { // Example command to request UID
      const uint8_t *uid = rfid_uids[chip->uid_index];
      for (int i = 0; i < RFID_UID_LENGTH; i++)
      {
        pin_write(chip->miso_pin, uid[i]); // Send UID byte-by-byte
        printf("Sent byte %02X to MISO\n", uid[i]);
      }
      chip->uid_index = (chip->uid_index + 1) % RFID_UID_COUNT; // Cycle through UIDs
      printf("RFID UID sent: %02X %02X %02X %02X\n", uid[0], uid[1], uid[2], uid[3]);
    }
    else
    {
      printf("Unknown command received: %02X\n", command);
    }
  }
}