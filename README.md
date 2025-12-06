# SchaltMichAus

**SchaltMichAus** is a small smart add-on for my audio setup (Yamaha RX-V1065).  
It monitors the receiver’s activity and automatically powers down my additional hardware (Bluetooth receiver, ARC extractor) when no audio is playing.

**Goal:**  
Reduce energy consumption, avoid unnecessary idle time, and extend the lifetime of the receiver and connected devices – without changing how the setup is normally used.

## Motivation
I inherited a Yamaha RX-V1065 from my grandpa.  
Whenever someone in the flat listened to music in the evening, we often forgot to turn the amplifier off. Managing the peripherals manually or switching the TV audio output every time was inconvenient. I wanted a solution that:

- does not change how the receiver is used,
- requires no firmware or hardware modifications,
- and keeps the device as my grandpa left it.

## Features

- **Automatic idle detection**
  - Monitors the receiver’s 12 V trigger output.
  - Analyzes the audio-out signal to detect real audio activity.
  - Marks the system as “inactive” after a configurable silence timeout.

- **Automatic power-down**
  - Powers down connected devices via a load switch.
  - Sends IR commands to the receiver to perform a clean shutdown, just like a remote.

- **Non-intrusive integration**
  - Installed between the receiver’s audio/trigger outputs and the peripherals.
  - Requires no modification of the receiver.
  - Works completely in the background.

- **Smart audio detection**
  - Uses a DC-biased input and statistical deviation (“loudness over time”) to detect meaningful audio.
  - Avoids false shutdowns during quiet passages or short pauses.

## Typical Use Case

1. I turn on the receiver as usual.  
2. SchaltMichAus detects the active state and begins monitoring audio.  
3. During playback, all devices remain powered.  
4. After playback stops and the configured silence interval passes:  
   - Peripherals power down.  
   - The receiver is turned off via IR.  

**Result:**  
No more receiver running for hours after someone forgot to turn it off.

## Hardware Overview

- **Controller:** ESP32 (AZ-Delivery version)  
- **Inputs:**  
  - 5 V via USB-C  
  - 12 V trigger input  
  - Audio-out signal (approx. ±1 V)  
- **Outputs:**  
  - IR LED for power commands  
  - Load switch for Bluetooth module, USB devices, etc.  

## Status

- IR on/off control: **working**  
- Trigger signal monitoring: **working**  
- Audio-based idle detection: **working**  
- Custom PCB with ESP32, audio and trigger inputs, load switch: **working** (with fixes applied)

## Initial Problems

- The first ESP32 unit was unreliable, causing inconsistent behaviour → resolved by replacing it.  
- Audio detection pin was not ADC-capable → fixed by soldering a jumper wire to a valid ADC pin.
