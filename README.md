# Beatrice

**Beatrice** is a real-time beat detection system for the **ESP32**. It captures audio from a microphone via **I2S** and drives LEDs in sync with the beat. It was originally created to control red & blue LEDs on a custom-made hat for a DJ with a "space cop" theme.

> **Beatrice**: a pun on *ricing* (**slang:** adding LEDs to things) and "beat" + "rice".

### Demo Video

<p align="center">
  <a href="https://www.youtube.com/watch?v=R6wnha9A4r0">
    <img src="https://i.ibb.co/G4L1pCs5/beatrice.gif" alt="Demo video"/>
  </a>
</p>

The system is **multithreaded with FreeRTOS**, with dedicated tasks for microphone input and beat detection. The beat detection algorithm begins with a simple energy-based peak detector, but operates on the **magnitude spectrum** (via **Kiss FFT**) rather than raw time-domain energy. Frequency-domain energy is **weighted** to emphasize ranges typically associated with bass and snare drums. Several running averages and adaptive thresholds are used to track dynamic changes in the input audio.

To detect **harmonic shifts** (e.g., root note changes), the system applies ideas from **Hilbert space theory**. Specifically:

> It computes the "angle" between the current spectral frame and a running average of previous frames using a **weighted inner product** on L²(ℤ/256ℤ), with weights chosen to isolate bass frequencies. An angle near ½π indicates a significant change (e.g. root note transition), and this angle is used to boost the activation strength of such frames.

Notably, **Beatrice** works quite well with "noisier" genres such as rock and metal, where basic energy-based detectors struggle. See [here](https://youtu.be/ZtbcuLpQV-4) for a short demonstration.

## Requirements

- [ESP-IDF v5+](https://github.com/espressif/esp-idf)
- Kiss FFT (included)
- Python (for flashing)

## Build and Flash

To build and flash, run

```
idf.py build flash
```

in the root folder with an ESP32 board connected.

## Pins

The pin configuration is hard-coded via macros in [led_control.h](main/led_control.h) and [mic.h](main/mic.h). These can be modified freely to suit your set-up.

## Compatibility

In theory, any ESP32 board will work. Known working boards are

- Freenove ESP32-WROOM

- Seeed Studio XIAO ESP32S3

In testing on the WROOM, the beat detection thread was found to spend two thirds of its time waiting for the next buffer of samples to become available, so any ESP32 chip should do, although dual core is recommended.

Beatrice was written for, and tested with, an INMP441 microphone, but any microphone with an I2S interface can be used, so long as the I2S configuration is correct.
