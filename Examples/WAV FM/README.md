## WAV FM
- What it does?
    - Modulate the carrier frequency with samples taken from a wav file
- How it does?
    - Set pin 7 to use the clock
      - Pin 7 corresponds to GPIO4
      - Clock function is its Alternate Function 0 (GP_AF0 corresponds to clock GPCLK0)
    - Set clock frequency
      - Set PLLD clock as source of clock generator 0 (GPCLK0 is used by GPIO4)
      - Set a divisor of PLLD source frequency (500 MHz) to achieve the desired frequency
    - Modulate clock frequency
      - Get a sample from the wav file in floating form (each sample has a value from -1.0 to +1.0)
      - Multiplicate the sample with 75000 (the standard deviation of 75KHz when using 200KHz spacing)
      - Add the result to the carrier frequency
      - Wait some nanoseconds according to samplerate

### Power supply warning
This example modulates the carrier frequency using the CPU.  
Raspberry cannot sleep less than 100 microseconds, but with a samplerate of 44KHz we need to wait about 22 microseconds, so I implemented a busy waiting delay to achieve this.  
This form of waiting is very CPU intensive and if Raspberry is not supplied with enough current the CPU cloud be sometimes underclocked and you will ear your audio file reproduced slowly.  
To solve this be sure to use a power supply with enough current and use a short USB cable.  
With this two precautions I have avoided the dynamic underclock and my audio file is reproduced at the right samplerate.  

### C Usage
This example uses bcm_host.h to sopport any version of Raspberry Pi, so you have to compile it with `-l bcm_host`
- Compile
  - `gcc wav_fm.c -o wav_fm -O2 -L /opt/vc/lib -l bcm_host -I /opt/vc/include -lsndfile -lm`
- Run
  - Default frequency of 105.0 MHz
    - `sudo nice -n -20 ./wav_fm sound.wav`
  - Frequency in hertz (106.2 MHz)
    - `sudo nice -n -20 ./wav_fm sound.wav 106200000`
- Stop
  - ctrl-\
  - ctrl-c
  
