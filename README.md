# TF2 Sentry
This is a repository for the source code on the STM in the sentry.

# Description
This project is a working real life replica of the in-game sentry 
from Team Fortress 2. It has an FOV of 90 degrees and scans within 
that range. When an object comes within a certain range, the sentry 
will turn to it and "fire", aka, play audio and perform a muzzle flash.
It will communicate with via bluetooth to the smartphone PDA app 
found [here](https://github.com/JAePortfolio/sentryPDA_App).

The STM will be used to operate the following 
* Ultrasonic Sensors
* Stepper Motor
* Bluetooth 
* DAC/Audio
* LEDs
* Laser

## Built With
* [STM32F7 Microcontroller](https://www.st.com/en/microcontrollers-microprocessors/stm32f7-series.html) - Microcontroller
* [C](https://en.wikipedia.org/wiki/C_(programming_language)) - Language Used

## Author
* **John Arena** - [JAeportfolio](https://github.com/JAePortfolio)

## License
This project is NOT open source.
