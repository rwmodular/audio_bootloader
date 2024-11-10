# RP2040 Audio Bootloader
You can update the firmware on most of the Mutable Instruments eurorack modules by playing an audio file on your computer/phone/tablet and connecting an audio cable to one of the CV input sockets.

This project describes how to get the Mutable Instruments bootloader running on an RP2040 by combining it with code from the <a href="https://vanhunteradams.com/Pico/Bootloader/Bootloader.html">Serial Bootloader</a> project from V. Hunter Adams. The Serial Bootloader project gives an excellent overview of the RP2040 boot process.

This code hasn't been tested as-is but is based on working code used on a RP2040 based eurorack module. The CV input circuit scales a -5v to +5v input into a 0v to 3.3V using a similar layout to that found in most Mutable Instrument schematics.

If you use this in your module, have any questions or suggestions, please get in touch.

## Bootloader
The bootloader will launch the firmware unless there is no firmware in flash or conditions are met to enter programming mode (e.g. a button is held). In programming mode it will wait for an audio signal, decode it and write it to flash and then start the firmware after the transfer has ended.

First we add a linker script to our project based on the <a href="https://github.com/raspberrypi/pico-sdk/blob/1.5.1/src/rp2_common/pico_standard_link/memmap_default.ld">default RP2040 one</a>. Change this line in the script so that the bootloader only uses the first 32K of flash:
``` 
    FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 32K
```

In the CMakeLists.txt file, add this line to use the linker script:
```
pico_set_linker_script(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/bootloader_linker_script.ld)
```

The code for the bootloader is almost entirely copied from the Mutable Instruments bootloader.

We start an ADC interrupt running at 48KHz to sample the RP2040 ADC and add the sample (0 to 4095) to the demoulator.

By working with a page size of 4096 bytes we can erase and write the whole sector. We just need to make sure that current_address is initialized to our firmware start point (32K).
```
// write a block of new firmware to flash
void programPage(const uint8_t* data, size_t size) {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(current_address, size);
    flash_range_program(current_address, data, size);
    restore_interrupts (ints);

    current_address += size;
}
```

The code to start our firmware comes from the Serial Bootloader project and is based on the pico sdk code from crt0.S.

```
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
```

## Firmware
The firmware code needs very little modification to get it to run with the bootloader. All you need to do is copy the memmap_default.ld linker script as your did for the bootloader and reference it in the CMakeLists.txt file.

In the MEMORY section, change the FLASH to start after our bootloader and change the LENGTH to be the full size of flash minus the bootloader size:

``` 
    FLASH(rx) : ORIGIN = 0x10000000 + 32K, LENGTH = 2048k - 32K
```

Then stop the linker from adding the default second stage bootloader by removing the following section from the linker script:

```
    .boot2 : {
        __boot2_start__ = .;
        KEEP (*(.boot2))
        __boot2_end__ = .;
    } > FLASH

    ASSERT(__boot2_end__ - __boot2_start__ == 256,
        "ERROR: Pico second stage bootloader must be 256 bytes in size")
```

If you have a USB port on your RP2040 board you can drop the bootloader .uf2 file onto it followed by the firmware .uf2 rather than waiting for the audio transfer. This can be helpful when preparing multiple units for retail.

To debug your firmware, just comment out the link from CMakeLists.txt to it runs as the only code in the flash.

## Preparing the audio file
The Mutable Instruments bootloader project codes with a python script to generate a .wav file from the compiled firmware. You will need Python 2.x to run it with NumPy installed.

In the QPSK encoder.py, an RP2040 profile has been added to work in 4096 byte pages with a 3 second pause between each page.

Convert your firmware .bin file to a .wav using:
``` 
python encoder.py -t rp2040 -k -s 48000 -b 12000 -c 6000 -p 256 "path/to/pico_firmware.bin"
```

## Notes
If the -k flag when running encoder.py to 'scramble', the second parameter in the following line of code in the bootloader must be 'true':
```
decoder.Init(20000, true);
```

The 20000 timeout in the first parameter and the 3 second pause between blocks have been found to work with the RP2040 but I haven't spend much time looking into why or if they could/should be changed.
