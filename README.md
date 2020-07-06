# inverted-pendulum-stm32nucleo

### Project Objective:
Impedance Control of an Inverted Pendulum actuated with a DC motor and controlled with an STM32F746G Nucleo board.

## Components
- Nucleo Board STM32F746G
- Custom PCB board with H-Bridge
- DC motor: Maxon 402912
- Rotation encoder HEDS 5540
- Pendulum

## Simulink Model
The impedance controller is designed with two cascade controllers for position and velocity and a feedforward term to compensate the gravity.
![image](https://user-images.githubusercontent.com/41896432/86565828-db999180-bf68-11ea-9215-6d40b571ef43.png)

## Implementation
The encoder signal is read using a timer in encoder mode.

Through Simulink Coder, C code is generated and used in the SysTick interrupt of the microcontroller to compute a voltage input, given the encoder readings.

The control voltage is transformed into a PWM duty cycle to control the motor and another timer generates the 20KHz signal supplied to the H-Bridge.
