// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// Based on Mutable Instruments bootloaders Copyright 2016 Emilie Gillet.
// With RP2040 bootlader code from V. Hunter Adams - https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/Bootloaders/Serial_bootloader
//
// Tom Waters 2024

#include <stdio.h>
#include <cstring>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "qpsk/packet_decoder.h"
#include "qpsk/demodulator.h"
using namespace stm_audio_bootloader;

// where the firmware starts (32K reserved for bootloader)
#define FIRMWARE_START_ADDR (32 * 1024)
#define PIN_ADC1    26

const double kSampleRate = 48000.0;
const double kModulationRate = 6000.0;
const double kBitRate = 12000.0;
const uint16_t kPacketsPerPage = FLASH_SECTOR_SIZE / kPacketSize;

enum UiState {
  UI_STATE_WAITING,
  UI_STATE_RECEIVING,
  UI_STATE_ERROR,
  UI_STATE_WRITING
};

PacketDecoder decoder;
Demodulator demodulator;

int discard_samples = 8000;
int32_t peak = 0;
int32_t gain_pot = 0;
uint32_t current_address;
uint16_t packet_index;
uint8_t rx_buffer[FLASH_SECTOR_SIZE];

volatile UiState ui_state;

// Address of binary information header
uint8_t *flash_target_contents = (uint8_t *) (XIP_BASE + FIRMWARE_START_ADDR + 0xD4);

// Set VTOR register, set stack pointer, and jump to the reset
// vector in our application. Basically copied from crt0.S.
static inline void handleBranch() {

    // In an assembly snippet . . .
    // Set VTOR register, set stack pointer, and jump to reset
    asm volatile (
    "mov r0, %[start]\n"
    "ldr r1, =%[vtable]\n"
    "str r0, [r1]\n"
    "ldmia r0, {r0, r1}\n"
    "msr msp, r0\n"
    "bx r1\n"
    :
    : [start] "r" (XIP_BASE + FIRMWARE_START_ADDR), [vtable] "X" (PPB_BASE + FIRMWARE_START_ADDR)
    :
    );
}

// check if we have any firmware
bool hasFirmware() {
    return (flash_target_contents[0] == 0xF2) && (flash_target_contents[1] == 0xEB) && 
        (flash_target_contents[2] == 0x88) && (flash_target_contents[3] == 0x71);
}

// check if we should enter programming mode
// (add checks for button combos or pot positions to check at boot)
bool enterProgrammingMode() {
    return false;
}

// show the current progress
inline void updateLeds() {
    // Show status info on the 4 LEDs.
    absolute_time_t t = get_absolute_time();
    uint32_t now = to_ms_since_boot(t);
    switch (ui_state) {
        case UI_STATE_WAITING:
        {
            bool on = now & 128;
            gpio_put(PICO_DEFAULT_LED_PIN, on);
        }
        break;

        case UI_STATE_RECEIVING:
        case UI_STATE_WRITING:
        {
            bool on = now & 512;
            gpio_put(PICO_DEFAULT_LED_PIN, on);
        }
        break;

        case UI_STATE_ERROR:
        {
            gpio_put(PICO_DEFAULT_LED_PIN, true);
        }
        break;
    }
}

// write a block of new firmware to flash
void programPage(const uint8_t* data, size_t size) {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(current_address, size);
    flash_range_program(current_address, data, size);
    restore_interrupts (ints);

    current_address += size;
}

void fillBuffer() {
    uint16_t sample = adc_fifo_get_blocking();

    if (!discard_samples) {
        demodulator.PushSample(sample);
    } else {
        --discard_samples;
    }

    updateLeds();
}

void initializeReception() {
  decoder.Init(20000, true);
  demodulator.Init(
      kModulationRate / kSampleRate * 4294967296.0,
      kSampleRate / kModulationRate,
      2.0 * kSampleRate / kBitRate);
  demodulator.SyncCarrier(true);
  decoder.Reset();
  current_address = FIRMWARE_START_ADDR;
  packet_index = 0;
  ui_state = UI_STATE_WAITING;
}

// Init things we always need
void initAll() {
    // init anything we always need
    // e.g. buttons we use to check if we should enter programming mode
}

// Init things we need to update the firmware
void initFlash() {
    // init led
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // init adc
    adc_init();
    adc_gpio_init(PIN_ADC1);
    adc_set_round_robin(0x01);
	adc_fifo_setup(true, true, 1, false, false);
	adc_set_clkdiv((48000000 / 48000) - 1); // 48KHz interrupt
    adc_select_input(0);
	irq_set_exclusive_handler(ADC_IRQ_FIFO, fillBuffer);
	adc_irq_set_enabled(true);
	irq_set_enabled(ADC_IRQ_FIFO, true);
	adc_run(true);
}

// Cleanup after flashing before starting the firmware
void deInit() {
    adc_run(false);
    irq_set_enabled(ADC_IRQ_FIFO, false);
    adc_irq_set_enabled(false);

    gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

int main() {
    stdio_init_all();
    initAll();

    if(enterProgrammingMode() || !hasFirmware()) {     
        initFlash();
        initializeReception();
        bool exit_updater = false;
        while (!exit_updater) {
            bool error = false;

            if (demodulator.state() == DEMODULATOR_STATE_OVERFLOW) {
                error = true;
            } else {
                demodulator.ProcessAtLeast(32);
            }

            while (demodulator.available() && !error && !exit_updater) {
                uint8_t symbol = demodulator.NextSymbol();
                PacketDecoderState state = decoder.ProcessSymbol(symbol);
                switch (state) {
                    case PACKET_DECODER_STATE_OK:
                        {
                            ui_state = UI_STATE_RECEIVING;
                            memcpy(
                                rx_buffer + (packet_index % kPacketsPerPage) * kPacketSize,
                                decoder.packet_data(),
                                kPacketSize);
                            ++packet_index;
                            if ((packet_index % kPacketsPerPage) == 0) {
                                ui_state = UI_STATE_WRITING;
                                programPage(rx_buffer, FLASH_SECTOR_SIZE);
                                decoder.Reset();
                                demodulator.SyncCarrier(false);
                                ui_state = UI_STATE_RECEIVING;
                            } else {
                                decoder.Reset();
                                demodulator.SyncDecision();
                            }
                        }
                        break;
                    case PACKET_DECODER_STATE_ERROR_CRC:
                    case PACKET_DECODER_STATE_ERROR_SYNC:
                        error = true;
                        break;
                        break;
                    case PACKET_DECODER_STATE_END_OF_TRANSMISSION:
                        exit_updater = true;
                        deInit();
                        break;
                    default:
                        break;
                }
            }

            if (error) {
                ui_state = UI_STATE_ERROR;
                // show the error state for a while before resetting
                // consider waiting here for a button press to try again
                for(int i=0; i<50; i++) {
                    sleep_ms(100);
                }
                initializeReception();
            }
        }
    }

    // start the firmware
	handleBranch();
    while(1) { }
}