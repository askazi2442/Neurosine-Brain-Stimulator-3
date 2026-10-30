# RP2040 DDS Slave

Dual-core Direct Digital Synthesis (DDS) signal generator for the RP2040.
Core 0 runs an I2C slave for remote control; Core 1 drives an AD5662 16-bit
SPI DAC to produce sine, square, triangle, and sawtooth waveforms with adjustable frequency and amplitude.

---

## Pin Connections

### I2C Control Bus (Core 0)

| RP2040 GPIO | Signal   | Notes                               |
|-------------|----------|------------------------------------|
| GPIO 6      | I2C1 SDA | Pull up to 3.3 V (4.7 kΩ typical)  |
| GPIO 7      | I2C1 SCL | Pull up to 3.3 V (4.7 kΩ typical)  |

Bus speed: 400 kHz (Fast-mode). External pull-ups are required — the firmware
enables internal pull-ups only on the SPI SYNC line, not on the I2C lines.

---

### AD5662 DAC (Core 1, SPI0)

| RP2040 GPIO | AD5662 Pin   | Signal                           |
|-------------|--------------|----------------------------------|
| GPIO 4     | PIN 1 (SYNC) | Active-low chip select — SYNC  |
| GPIO 2     | PIN 3 (SCLK) | SPI clock (20 MHz)             |
| GPIO 3     | PIN 4 (SDIN) | SPI data (MOSI)                |

**Supply and reference:**
* VDD (PIN 5): connect to **3.3 V** (required for the 2 mA full-scale mapping — see below).
* GND (PIN 2 / AGND): connect to system ground.
* VOUT (PIN 6): analog output (0 V to 3.3 V).
* The AD5662 has no separate VREF pin; its full-scale output equals VDD.

**SPI mode:** CPOL = 0, CPHA = 1 (Mode 1). Data is clocked in MSB-first on
the falling edge of SCLK while SYNC is held low.

---

### I2C Address Selection (GPIO 10 / 11 / 12)

Three GPIO pins select one of eight 7-bit I2C addresses (0x50–0x57):

| GPIO 12 (A2) | GPIO 11 (A1) | GPIO 10 (A0) | I2C Address |
|:------------:|:------------:|:------------:|:-----------:|
| LOW          | LOW          | LOW          | 0x50        |
| LOW          | LOW          | HIGH         | 0x51        |
| LOW          | HIGH         | LOW          | 0x52        |
| LOW          | HIGH         | HIGH         | 0x53        |
| HIGH         | LOW          | LOW          | 0x54        |
| HIGH         | LOW          | HIGH         | 0x55        |
| HIGH         | HIGH         | LOW          | 0x56        |
| HIGH         | HIGH         | HIGH         | 0x57        |

**Wiring rules:**
* Each address pin has an internal pull-down enabled in firmware.
* **Leave the pin unconnected (or tie to GND)** to set that bit = 0.
* **Tie the pin to 3.3 V through a 1 kΩ resistor** to set that bit = 1.
* Never drive an address pin directly from a GPIO output on another device without a series resistor; the internal pull-down is weak (~50–80 µA).

---

## DAC Output Scaling (V-to-I)

The AD5662 VDD must be **3.3 V**. The external V-to-I stage is sized so that
the full-scale DAC voltage (3.3 V) produces the maximum output current of
**2 mA** in a unipolar configuration.

| DAC Code | VOUT    | Output Current |
|----------|---------|----------------|
| 0x0000   | 0.0 V   | 0 mA           |
| 0x4000   | 0.825 V | 0.5 mA         |
| 0x8000   | 1.65 V  | 1.0 mA         |
| 0xC000   | 2.475 V | 1.5 mA         |
| 0xFFFF   | 3.3 V   | 2.0 mA (max)   |

**Implied load resistance:** R = V / I = 3.3 V / 2 mA = **1650 Ω**

For a simple passive V-to-I conversion, place a 1650 Ω resistor between VOUT
and GND. For a precision implementation, use a Howland current pump or an op-amp voltage-to-current converter.

**Amplitude Control:** The DDS firmware utilizes direct Q16 fixed-point math to scale the waveform amplitude dynamically based on the requested peak current (in µA) commanded over I2C.

---

## Startup Calibration

On every power-on or reset, Core 1 runs `spi_dac_calibrate()` before
starting the DDS engine:

1.  **Power-cycle** — the DAC is put into Hi-Z power-down mode for 2 ms, then woken back to normal operation at 0mA. This resets any residual output state.
2.  **Five-point ramp sweep** — the following codes are written in sequence, each held for 1 ms:

    | Step | DAC Code | VOUT    | Current |
    |------|----------|---------|---------|
    | 1    | 0x0000   | 0.0 V   | 0 mA    |
    | 2    | 0x4000   | 0.825 V | 0.5 mA  |
    | 3    | 0x8000   | 1.65 V  | 1.0 mA  |
    | 4    | 0xC000   | 2.475 V | 1.5 mA  |
    | 5    | 0xFFFF   | 3.3 V   | 2.0 mA  |
    | 6    | 0x0000   | 0.0 V   | 0 mA    |

The DAC output is left safely at **0x0000 (0 mA)** after calibration. The staircase ramp is visible on an oscilloscope during bring-up and confirms that SPI communication is working before the DDS loop takes over.

---

## I2C Command Protocol

The RP2040 acts as an I2C slave listening for specific command bytes followed by their payloads. All multi-byte fields are transmitted **Little-Endian** (LSB first).

| Cmd Byte | Command Name    | Payload | Description                                                                 |
|----------|-----------------|---------|-----------------------------------------------------------------------------|
| `0x01`   | SET_FREQ        | 4 bytes | Set frequency in **mHz** (milliHertz) as a `uint32_t`. (e.g., 1000000 = 1kHz) |
| `0x02`   | SET_WAVEFORM    | 1 byte  | `0` = Sine, `1` = Square, `2` = Triangle, `3` = Sawtooth                    |
| `0x03`   | SET_AMPLITUDE   | 2 bytes | Set peak amplitude in **µA** (microamps) as a `uint16_t`. (e.g., 2000 = 2mA) |
| `0x04`   | SET_ENABLE      | 1 byte  | `0` = Output Off (0mA idle), `1` = Output On                                |
| `0x05`   | SET_ALL         | 8 bytes | Sequentially updates Freq (4), Amp (2), Wave (1), and Enable (1).           |
| `0xFF`   | RESET           | 0 bytes | Resets the DDS engine to 1kHz, 1mA, Sine, Output Off.                       |

### Status Readback (GET_STATUS)
To read the current state of the DDS engine (shadow registers):
1.  **Write:** Send command byte `0x06`
2.  **Read:** Perform an I2C read request for **9 bytes**. 

The RP2040 will return:
`[Freq_LSB, Freq_2, Freq_3, Freq_MSB, Amp_LSB, Amp_MSB, Waveform, Enable, Fault_Code]`

---

## Build

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk).

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build
cmake ..
make -j$(nproc)
