# Nereo Navigator
This Repository contains the firmware used in Nereo's navigator board.
The board packs an STM32F469VIT6 microcontroller, and the project is configured using STM32CUBEMX, and later on compiled with arm-none-eabi.
## Using uROS
The uROS library is used to communicate with the microcontroller, in fact it should be connected with serial to a microROS Agent.
You can find all the instructions on how to use microROS on any STM32 MCU on uROS website, as I did.
## Using this project
This project has been designed to run on an STM32F469VIT6 MCU, yet it could in theory run (with proper adjustments) on any STM32 MCU: be careful tho, as it needs more than 300kB of SRAM for running, so consider using external SRAM.