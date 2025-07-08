#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RFID_UID_COUNT 10
#define RFID_UID_LENGTH 4

const uint8_t rfid_uids[RFID_UID_COUNT][RFID_UID_LENGTH] = {
    {0x12, 0x34, 0x56, 0x78}, // Card 1
    {0xAB, 0xCD, 0xEF, 0x01}, // Card 2
    {0x23, 0x45, 0x67, 0x89}, // Card 3
    {0x9A, 0xBC, 0xDE, 0xF0}, // Card 4
    {0x11, 0x22, 0x33, 0x44}, // Card 5
    {0x55, 0x66, 0x77, 0x88}, // Card 6
    {0x99, 0xAA, 0xBB, 0xCC}, // Card 7
    {0xDD, 0xEE, 0xFF, 0x00}, // Card 8
    {0x13, 0x57, 0x9B, 0xDF}, // Card 9
    {0x24, 0x68, 0xAC, 0xE0}  // Card 10
};

typedef struct
{
    uint32_t uid_index;
    pin_t miso_pin;
    pin_t sck_pin;
    pin_t mosi_pin;
    pin_t sda_pin;
    pin_t rst_pin;
    uint32_t detection_counter;
    bool card_present;
} chip_state_t;

static void chip_timer_event(void *user_data);

void chip_init()
{
    chip_state_t *chip = malloc(sizeof(chip_state_t));
    memset(chip, 0, sizeof(chip_state_t));

    // Setup SPI pins
    chip->mosi_pin = pin_init("MOSI", INPUT);
    chip->miso_pin = pin_init("MISO", OUTPUT);
    chip->sck_pin = pin_init("SCK", INPUT);
    chip->sda_pin = pin_init("SDA", INPUT_PULLUP);
    chip->rst_pin = pin_init("RST", INPUT_PULLUP);

    // Initialize state
    chip->uid_index = 0;
    chip->detection_counter = 0;
    chip->card_present = false;

    // Setup timer to simulate RFID card detection
    const timer_config_t timer_config = {
        .callback = chip_timer_event,
        .user_data = chip};
    timer_t timer = timer_init(&timer_config);
    timer_start(timer, 3000000, true); // Trigger every 3 seconds

    printf("RFID RC522 simulation started with %d cards.\n", RFID_UID_COUNT);
}

static void chip_timer_event(void *user_data)
{
    chip_state_t *chip = (chip_state_t *)user_data;

    // Simulate card detection/removal cycles
    chip->detection_counter++;

    // Simulate card presence pattern: present for 2 cycles, absent for 1 cycle
    if (chip->detection_counter % 3 == 0)
    {
        chip->card_present = false;
        printf("RFID: No card detected\n");
        return;
    }
    else
    {
        chip->card_present = true;

        // Cycle through different cards
        if (chip->detection_counter % 3 == 1)
        {
            chip->uid_index = (chip->uid_index + 1) % RFID_UID_COUNT;
        }
    }

    // Check if SDA is active (chip selected) and there's a card present
    if (pin_read(chip->sda_pin) == 0 && chip->card_present)
    {
        printf("RFID: Card detected, SPI communication active\n");

        // Simulate SPI communication
        const uint8_t *uid = rfid_uids[chip->uid_index];

        // Send UID data via MISO
        for (int i = 0; i < RFID_UID_LENGTH; i++)
        {
            pin_write(chip->miso_pin, uid[i]);
        }

        printf("RFID: Card %d detected - UID: %02X %02X %02X %02X\n",
               chip->uid_index + 1, uid[0], uid[1], uid[2], uid[3]);
    }
    else if (chip->card_present)
    {
        // Card is present but not being read
        const uint8_t *uid = rfid_uids[chip->uid_index];
        printf("RFID: Card %d available - UID: %02X %02X %02X %02X\n",
               chip->uid_index + 1, uid[0], uid[1], uid[2], uid[3]);
    }
}
