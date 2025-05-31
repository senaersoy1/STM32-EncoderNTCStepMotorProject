# 🔬 STM32-EncoderNTCStepMotorProject

This project demonstrates a simplified CT scanner control system using an STM32F103RB Nucleo board, a rotary encoder, an NTC thermistor, and a stepper motor. The system mimics motion and temperature control mechanisms of real CT scanners, focusing on safety-oriented automation.

## 📦 Hardware Used

- STM32F103RB Nucleo Board
- Rotary Encoder
- NTC Thermistor
- Stepper Motor
- L298N Motor Driver

## 🧠 System Overview

- A rotary encoder simulates gantry rotation. One full rotation equals 20 encoder ticks.
- An NTC thermistor monitors simulated X-ray tube temperature.
- After detecting 20 encoder ticks, the stepper motor moves the patient table forward (¼ step).
- If temperature exceeds 80°C, the system pulls the table backward (4 full turns) to simulate emergency patient ejection.

## ⚙️ Features Implemented

- Encoder counting via external interrupt (EXTI)
- Temperature sensing via ADC interrupt
- Stepper motor control with custom microsecond delay
- Overheat protection using flag logic to avoid repeated triggering
- All real-time events handled using interrupt service routines

## 🛠️ Implementation Notes

- `HAL_Delay()` was replaced with a timer-based delay for precise stepper motor control.
- System uses a flag to ensure the backward motion is triggered*only once when the temperature threshold is crossed.
- Interrupt-driven design ensures responsiveness outside the `while(1)` loop.

## 📷 Demonstration Logic

- 🔄 **20 encoder ticks** → table moves forward ¼ rotation.
- 🌡️ **NTC > 80°C** → table moves backward 4 full rotations.
- Interrupt-driven feedback control handles encoder and temperature events.

## 📁 Project Structure
STM32-EncoderNTCStepMotorProject/
├── Core/ # Application source and headers
├── Drivers/ # STM32 HAL drivers
├── .ioc # STM32CubeMX configuration file
├── README.md # This file
└── (ignored) Debug/, build/, etc.

## Author

Sena Ersoy – Istanbul Technical University  
ELK346 Term Project
