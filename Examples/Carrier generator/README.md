# Examples

Here you can find some basic examples to understand how RPiRadio works

## Usage
These examples use bcm_host.h to sopport any version of Raspberry Pi, 

so you have to compile them with `-l bcm_host`

### Examples list
- carrier_generator.c
  - What it does?
      - Only create a carrier on passed frequency or on default frequency 104.5 MHz
  - How it does?
      - Set pin 7 to use the clock
        - Pin 7 corresponds to GPIO4
        - Clock function is its Alternate Function 0 (GP_AF0 corresponds to clock GPCLK0)
      - Set clock frequency
        - Set PLLD clock as source of clock generator 0 (GPCLK0 is used by GPIO4)
        - Set a divisor of PLLD source frequency (500 MHz) to achieve the desired frequency
  - Compile
    - `gcc carrier_generator.c -o carrier_generator -L /opt/vc/lib -l bcm_host -I /opt/vc/include`
  - Run
    - Default frequency of 104.5 MHz
      - `sudo ./carrier_generator`
    - Frequency in hertz (106.2 MHz)
      - `sudo ./carrier_generator 106200000`
  - Stop
    - ctrl-\
    - ctrl-c

      
      
      
      
      

