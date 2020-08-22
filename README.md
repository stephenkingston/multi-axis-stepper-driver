# Multi-axis coordinated control of multiple stepper motors with acceleration and deceleration

Multi-axis simultaneous control of stepper motors is an unexpectedly hard problem (atleast it was a surprise to me at first), particularly when you are sending PWM pulses to each stepper motor driver to make a step.

This motor driver software takes absolute step input for all three motors via USB VCP from a computer and moves to said position while starting and stopping simultaneously. The driver software automatically takes care of the speed at which each stepper motor needs to move in order to achieve this.


# Demo 
![Demo](media/Coord-steppers.gif)

# Project
This is an STM32Cube project. Download STM32Cube from [ST's website.](https://www.st.com/en/development-tools/stm32cubeide.html)
