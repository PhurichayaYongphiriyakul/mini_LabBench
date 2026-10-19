# ELEC2104 TestBench — NUCLEO-L552ZE-Q as a mini lab

Firmware that turns an STM32 **NUCLEO-L552ZE-Q** into a signal generator, two-channel
oscilloscope and Bode analyser, used to test the Stage II band-pass filter/amplifier of
the ELEC2104 (University of Sydney) pulse oximeter project without lab equipment.

| Lab instrument | Replaced by |
| --- | --- |
| GW Instek GPS-4303 power supply | Nucleo 5V pin (USB) |
| Rigol DG1062Z function generator | DAC1_OUT1 (PA4) sine + 20 kΩ / 100 Ω attenuator |
| R&S RTB2002 oscilloscope | ADC1 on two channels + software lock-in amplifier |

![Measured Bode plot](docs/stage2_bode.png)

## Result (Stage II)

| Quantity | Measured | Theory |
| --- | --- | --- |
| Peak gain | 49.3 V/V (33.9 dB) at 1.5 Hz | 57 V/V (1 + R2/R3) |
| f_L (−3 dB) | 0.38 Hz | 0.48 Hz (1 / 2πR1C1) |
| f_H (−3 dB) | ~7.3 Hz | 6.0 Hz (1 / 2πR2C2) |

Components: R1 = 33 kΩ, C1 = 10 µF, R2 = 56 kΩ, R3 = 1 kΩ, C2 = 470 nF, LM358 on 5 V.

## How it works

- **TIM6** interrupts at exactly 2 kHz (110 MHz, PSC 109, ARR 499).
- Each tick the ISR reads ADC rank 1 (source sine, PC0) and rank 2 (op-amp output, PA3),
  then writes the next sample of a 256-entry sine look-up table (DDS) to the DAC.
- A **lock-in** multiplies both signals by sin and cos of the generator phase and averages
  over whole cycles, giving amplitude and phase at that frequency only. Gain is the
  ratio of the two channels, so DAC/ADC reference errors cancel.
- The firmware sweeps 17 frequencies (0.2–100 Hz), finds the −3 dB points and prints a
  machine-readable `DATA,...` block that `plot_bode.py` turns into a plot and CSV.

## Wiring

```
PA4 ──┬──[20 kΩ]──●──[100 Ω]── GND
      │           │ node X (~5 mVpp sine on ~0 V)
      └─→ A5 (PC0)│
                 (−) C1 (+) ── LM358 pin 3 ── R1 ── V_ref (~1.2 V)

LM358 pin 1 ──[10 kΩ]──●──[10 kΩ]── GND
                       └─→ A0 (PA3)

Nucleo 5V → LM358 pin 8, all grounds common
```

PA4 is the unlabelled hole next to D13 on CN7 (pin 9). D13 itself is PA5.

## CubeMX configuration

| Peripheral | Settings |
| --- | --- |
| Project | NUCLEO-L552ZE-Q, TrustZone disabled, toolchain CMake |
| DAC1 | OUT1 + OUT2, normal mode, buffer enabled, trigger none |
| ADC1 | IN1 + IN8 single-ended, 12-bit, synchronous clock ÷4, scan 2 ranks, 47.5 cycles, EOC single conversion, overrun data overwritten |
| TIM6 | PSC 109, ARR 499, global interrupt enabled (priority below SysTick) |
| LPUART1 | PG7/PG8, 115200 8N1 |
| SYS | Serial Wire debug, SysTick timebase |

Add to the end of `CMakeLists.txt` so `printf("%f")` works:

```cmake
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
```

If you change the attenuator or divider resistors, update `R_SER`, `R_SH` and `OUT_DIV`
in `Core/Src/main.c` (measured values give the most accurate gain).

## Build and flash

Requires STM32CubeCLT (arm-none-eabi-gcc, CMake, Ninja, STM32_Programmer_CLI).

```bash
cmake --preset Debug
cmake --build build/Debug
STM32_Programmer_CLI -c port=SWD -w build/Debug/ELEC2104_TestBench.elf -v -rst
```

## Use

```bash
pip3 install pyserial matplotlib
python3 -m serial.tools.miniterm /dev/tty.usbmodemXXXX 115200
```

| Key | Action |
| --- | --- |
| `d` | DC check: op-amp output with no signal (expect ≈ V_ref) |
| `s` | Bode sweep 0.2–100 Hz (~2 min) |
| `+` / `-` | Double / halve input amplitude (use `-` if `CLIP!` appears) |

To save the plot, exit miniterm (Ctrl + ]) and run:

```bash
python3 plot_bode.py /dev/tty.usbmodemXXXX
```

This writes `stage2_bode.png` and `stage2_bode.csv`.

## Repository layout

```
Core/Src/main.c        application code (inside USER CODE blocks)
ELEC2104_TestBench.ioc CubeMX project
CMakeLists.txt         build (with the printf-float line)
plot_bode.py           triggers a sweep over serial and plots the result
docs/stage2_bode.png   measured Bode plot
```
