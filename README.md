# dishwasher-beep-detector
Implementation of the [Goertzel algorithm](https://en.wikipedia.org/wiki/Goertzel_algorithm) to detect the end of cylce tone from my dishwasher.

The STM32F103 (bluepill) samples audio at 72 kHz, 1024 samples a time, and then tries to detect if a 4.9 kHz tone is present (my BOSCH dishwasher end of cylce beep).

## More information on [the hackaday.io project](https://hackaday.io/project/172112-automatic-dishwasher-opener)
