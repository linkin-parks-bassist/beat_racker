# Beatrice

**Beatrice** is a real-time beat detection system for the **ESP32**, written using **ESP-IDF**. It captures audio from an **INMP441** digital MEMS microphone, performs FFT-based analysis to detect rhythmic transients, and drives LEDs in sync with the beat. It was originally created for a friend to control red and blue LEDs on a custom-made police-themed DJ hat.

> **Beatrice**: a pun on *ricing* (**slang:** adding LEDs to things) and "beat" + "rice".

The system is **multithreaded with FreeRTOS**, with dedicated tasks for microphone input and beat detection. The beat detection algorithm begins with a simple energy-based peak detector, but operates on the **magnitude spectrum** (via **Kiss FFT**) rather than raw time-domain energy. Frequency-domain energy is **weighted** to emphasize ranges typically associated with bass and snare drums. Several running averages and adaptive thresholds are used to track dynamic changes in the input audio.

To detect **harmonic shifts** (e.g., root note changes), the system applies ideas from **Hilbert space theory**. Specifically:

> It computes the "angle" between the current spectral frame and a running average of previous frames using a **weighted inner product** on L²(ℤ/256ℤ), with weights chosen to isolate bass frequencies. An angle near ½π indicates a significant change (e.g. root note transition), and this angle is used to boost the activation strength of such frames.

### Build & Flash

To build and flash:

```
idf.py build
idf.py flash
```

Beatrice has been tested on

* Freenove ESP32-WROOM

* Seeed Studio XIAO ESP32S3
