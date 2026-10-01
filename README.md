# PulseShield v6

PulseShield is a real-time audio filter I’m building for people with LQT2, where sudden loud sounds can be a potential trigger for arrhythmic events.

The basic idea is pretty simple: detect a sudden spike in sound and bring it down really quickly, while leaving normal audio mostly unchanged. I’m trying to make the reduction fast enough to catch the spike but smooth enough that the audio doesn’t sound obviously distorted or chopped.

## How it works

The system runs sample-by-sample on a Teensy 4.0 with an INMP441 I2S microphone.

It first keeps track of the recent audio level and looks for sudden changes that are much louder than the surrounding sound. When it detects something that looks like a spike, it quickly reduces the gain. A short look-ahead buffer gives it a few milliseconds to react before the sound reaches the output, and the gain is then brought back up smoothly so the transition doesn’t feel as noticeable.

The original prototype was written in Python, and the current v6 implementation is in C++ for the Teensy.

## Current results

The results below are from a software simulation of the v6 C++ code using synthetic test sounds. I have **not** measured v6 on the Teensy yet.

Across 18 simulated harmful-event cases using alarms, claps, door slams, and feedback howl at different background levels:

- Minimum peak reduction: **18.9 dB**
- Median peak reduction: **29.0 dB**
- Maximum peak reduction: **39.5 dB**
- No startle-grade onset remained in any of the 18 cases

In a speech + sudden-event test:

- Clap: **16.8 dB reduction**
- Alarm: **26.0 dB reduction**
- Door slam: **5.9 dB reduction**
- Feedback howl: **25.5 dB reduction**

The door-slam number is lower because speech is already coming back through as the gain recovers. The first 5 ms of the event was still reduced to **0.047 of full scale**, compared with about **0.9 at the input**.

For normal audio, the effect was very small:

- Speech-like audio: **−0.1 dB**
- Quiet room → speech: **−0.9 dB**
- Slow swell: **0.0 dB**

A 6-beep, 3 kHz alarm-clock test was reduced from about **0.9** peak amplitude to **0.016–0.032** per beep.

The C++ implementation also matches the original Python reference to within **4e-5** in the current tests.

## Testing

I’ve tested v6 with:

- Speech with claps, alarms, door slams, and feedback howl
- Different background noise levels
- A 3 kHz alarm-clock beep train
- Speech, fast speech, a chord, a swell, and a steady tone
- Sudden swells building over 40 ms and 80 ms
- 10 minutes of loud noise followed by speech

## Hardware

Current setup:

- Teensy 4.0
- INMP441 I2S microphone
- Audio output through the Teensy audio system

The current algorithm has a **5 ms look-ahead**, which is the intended algorithmic latency.

I still need to measure the actual CPU usage and timing on the Teensy while running v6.

## Files

`pulseshield_teensy_v6_telemetry.ino` contains the current v6 Teensy implementation.

The `graphs_v6` folder contains plots from the simulations, and the audio folder contains a before/after example.

## A note on the project

This is still a prototype and the results so far are from software simulation. The goal right now is to validate the audio-processing approach and then test the same system on the actual Teensy hardware.

This project is not a medical device and these results do not show that it prevents arrhythmias. They are just the current results of the audio-filtering prototype.

More testing on hardware is the next step.
