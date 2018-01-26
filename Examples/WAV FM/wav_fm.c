/*
Goal:
    This example is for educational purposes only, and is fully commented
    This example only generate a carrier on passed frequency or on default frequency 105.0 MHz
    Turn your car radio on and check if there is silence on that frequency

Usage:
    Compile:
        gcc wav_fm.c -o wav_fm -O2 -L /opt/vc/lib -l bcm_host -I /opt/vc/include -lsndfile -lm
    Run:
        sudo nice -20 ./wav_fm audiofile.wav 
        sudo nice -20 ./wav_fm audiofile.wav 106200000
    Stop:
        ctrl-\
        ctrl-c

References:
    https://github.com/raspberrypi/documentation/raw/master/hardware/raspberrypi/bcm2835/BCM2835-ARM-Peripherals.pdf
        Section 6 General Purpose I/O (GPIO)
        Pages:
            -  90 (Register View)
            -  92 (GPIO Alternate function select register 0)
            - 105 (General Purpose GPIO Clocks, MASH dividers)
            - 107 (Clock Manager General Purpose Clock Control)
            - 108 (Clock Manager General Purpose Clock Divisors)
            - 172 (System Timer)

    https://github.com/raspberrypi/userland/blob/master/host_applications/linux/libs/bcm_host/bcm_host.c
        Broadcom functions to get physical addresses to allow compatibility on RPi 1 and successors:
            - unsigned bcm_host_get_peripheral_address(void)
            - unsigned bcm_host_get_peripheral_size(void)
            - unsigned bcm_host_get_sdram_address(void)

    https://pinout.xyz/pinout/gpclk
    https://pinout.xyz/pinout/pin7_gpio4

License:
    GPL3

Author:
    Lorenzo Santina (BigNerd95)
*/

#include <bcm_host.h>   // Broadcom library to support any version of RaspBerry
#include <stdio.h>      // Input/Output functions
#include <stdint.h>     // Integer types having specified widths
#include <unistd.h>     // Function pause to wait for signals
#include <fcntl.h>      // Function open
#include <sys/types.h>  // Sometimes needed by fcntl
#include <sys/stat.h>   // Sometimes needed by fcntl
#include <sys/mman.h>   // Function mmap

#include <sndfile.h>    // Wav sound library 
#include <stdlib.h>     // Function malloc
#include <strings.h>    // Function bzero
#include <math.h>       // Function round




/***********************
* Peripheral constants *
***********************/

// All the following are BUS addresses

// Peripheral BUS base address
#define BUS_BASE_ADDRESS 0x7E000000


// GPIO constants

#define FLAGS_PER_REGISTER 10    // There are 10 pin flags per register

// General Purpose I/O function select registers
const unsigned GPFSEL[] = {
    0x7E200000,     // GPFSEL0 register, BCM pin  0 to  9 (we use 4 [GPIO4 pin 7])
    0x7E200004,     // GPFSEL1 register, BCM pin 10 to 19 (not used in this example)
    0x7E200008,     // GPFSEL2 register, BCM pin 20 to 29 (not used in this example)
    0x7E20000C,     // GPFSEL3 register, BCM pin 30 to 39 (not used in this example)
    0x7E200010,     // GPFSEL4 register, BCM pin 40 to 49 (not used in this example)
    0x7E200014      // GPFSEL5 register, BCM pin 50 to 53 (not used in this example)
};

// GPIO functions flags (each flag is 3 bit long)
typedef enum {
    GP_INPUT  = 0b000,
    GP_OUTPUT = 0b001,
    GP_AF0    = 0b100,
    GP_AF1    = 0b101,
    GP_AF2    = 0b110,
    GP_AF3    = 0b111,
    GP_AF4    = 0b011,
    GP_AF5    = 0b010
} GP_FUNCTION;


// CLOCK constants

// Clock Manager General Purpose Clocks Control registers
const unsigned CM_GPCTL[] = {
    0x7E101070,    // CM_GP0CTL (GPCLK0) Control register [Alternative Function 0 (GP_AF0) of GPIO4 (pin  7)]
    0x7E101078,    // CM_GP1CTL (GPCLK1) Control register [Alternative Function 0 (GP_AF0) of GPIO5 (pin 29)] (not used in this example)
    0x7E101080     // CM_GP2CTL (GPCLK2) Control register [Alternative Function 0 (GP_AF0) of GPIO6 (pin 31)] (not used in this example)
};

// Clock Manager General Purpose Clock Divisors registers
const unsigned CM_GPDIV[] = {
    0x7E101074,    // CM_GP0DIV (GPCLK0) Divisor register [Alternative Function 0 (GP_AF0) of GPIO4 (pin  7)]
    0x7E10107C,    // CM_GP1DIV (GPCLK1) Divisor register [Alternative Function 0 (GP_AF0) of GPIO5 (pin 29)] (not used in this example)
    0x7E101084     // CM_GP2DIV (GPCLK2) Divisor register [Alternative Function 0 (GP_AF0) of GPIO6 (pin 31)] (not used in this example)
};

// Clock Managers General Purpose Clocks
typedef enum {
    CM_GPCLK0 = 0,
    CM_GPCLK1 = 1,
    CM_GPCLK2 = 2
} CM_GPCLK;

// Clock Manager flags
#define CM_PASSWD (0x5A << 24)
#define CM_ENAB   (1 << 4)
#define CM_BUSY   (1 << 7)

// Clock Manager Control register mash flags
typedef enum {
    CM_MASH_INT = (0 << 9),
    CM_MASH_1S  = (1 << 9),
    CM_MASH_2S  = (2 << 9),
    CM_MASH_3S  = (3 << 9)
} CM_MASH;

// Clock Manager Control register source flags
typedef enum {
  CM_GND           = 0,
  CM_OSCILLATOR    = 1,
  CM_TESTDEBUD0    = 2,
  CM_TESTDEBUG1    = 3,
  CM_PLLA          = 4,
  CM_PLLC          = 5,
  CM_PLLD          = 6,
  CM_HDMI          = 7
} CM_SRC;

#define PLLD_FREQ     500000000.00 // PLLD clock source frequency (500MHz)

// System timer
#define SYSTMR_CS     0x7E003000
#define SYSTMR_CLO    0x7E003004
#define SYSTMR_CHI    0x7E003008
#define SYSTMR_C0     0x7E00300C
#define SYSTMR_C1     0x7E003010
#define SYSTMR_C2     0x7E003014
#define SYSTMR_C3     0x7E003018

// Address conversion macro

unsigned VIRTUAL_BASE_ADDRESS = 0;       // Global variable where peripherals virtual base address will be contained after mapping physical address
#define CONVERT(address)    ((unsigned) (address) - BUS_BASE_ADDRESS + VIRTUAL_BASE_ADDRESS)  // convert BUS address to virtual address


/***********************
* Peripheral functions *
***********************/

// Map a physical address in the virtual address space of this process
void* map_memory(unsigned address, size_t size) {

    int fd = open("/dev/mem", O_RDWR | O_SYNC);     // Open /dev/mem to map the physical address
    if (fd < 0){                                    // Check if open is failed
        puts("Can't open /dev/mem\nRun as root!");
        exit(-1);
    }

    void* vaddr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, address);  // map physical address from /dev/mem fd with given size
    if (vaddr == MAP_FAILED){                       // Check if mmap is failed
        puts("Failed to map memory");
        exit(-1);
    }

    close(fd);                                      // Close /dev/mem fd
    return vaddr;                                   // Return mapped address in this process address space
}

// Maps the correct peripheral physical address in the virtual address space of this process
void map_peripheral(){
    unsigned peripheral_address = bcm_host_get_peripheral_address();  // Broadcom function to get peripheral physical address (so it support any Raspberry Pi version)
    unsigned peripheral_size = bcm_host_get_peripheral_size();        // Broadcom function to get peripheral size
    VIRTUAL_BASE_ADDRESS = (unsigned) map_memory(peripheral_address, peripheral_size);    // Map peripheral memory
}

// Map peripherals if they are not mapped
void check_peripheral(){
    if (!VIRTUAL_BASE_ADDRESS)
        map_peripheral();
}

// Set the value of the given BUS addres
void reg_set(unsigned address, uint32_t value){
    check_peripheral();             // Check if peripherals are mapped 
    (*(uint32_t*) CONVERT(address)) = value; // Set the value
}

// Get the value of the given BUS address
uint32_t reg_get(unsigned address){
    check_peripheral();             // Check if peripherals are mapped        
    return *(uint32_t*) CONVERT(address); // Get the value
}

// Set gpio pin function (pin number is in BCM format)
void set_gp_func(unsigned int pin, GP_FUNCTION function){
    if (pin <= 53){
        unsigned int reg_number = pin / FLAGS_PER_REGISTER;     // Calculate register number
        unsigned int pin_number = pin % FLAGS_PER_REGISTER;     // Calculate pin number of this register

        uint32_t value = reg_get(GPFSEL[reg_number]);           // Get GPFSELn register value
        value &= ~(   0b111 << 3 * pin_number);                 // Zeroes out the 3 bits relative to this pin
        value |=  (function << 3 * pin_number);                 // Set pin function
        reg_set(GPFSEL[reg_number], value);                     // Set GPFSELn register new value
    } else {
        puts("Invalid pin number!");
        exit(-1);
    }
}

// Disable clock generator and wait until busy flag turns off
void stop_clock(CM_GPCLK gpclk_number, CM_SRC clk_source){
    while (reg_get(CM_GPCTL[gpclk_number]) & CM_BUSY)              // Unless busy flag turns off
        reg_set(CM_GPCTL[gpclk_number], CM_PASSWD | clk_source);   // Disable clock generator
}

// Set frequency divisor, source, mash and enable clock generator
void start_clock(CM_GPCLK gpclk_number, CM_SRC clk_source, CM_MASH mash_stage){
    stop_clock(gpclk_number, clk_source);                                    // Disable clock generator before any changes
    reg_set(CM_GPCTL[gpclk_number], CM_PASSWD | mash_stage | clk_source);    // Set source and MASH

    while(!(reg_get(CM_GPCTL[gpclk_number]) & CM_BUSY))                                          // Unless busy flag turns on
        reg_set(CM_GPCTL[gpclk_number], CM_PASSWD | reg_get(CM_GPCTL[gpclk_number]) | CM_ENAB);  // Enable clock generator
}

// Set clock generator frequency
void set_clock_frequency(CM_GPCLK gpclk_number, uint32_t frequency){
    uint32_t clock_divisor = ((float)(PLLD_FREQ / frequency)) * (1 << 12);   // Calculate clock frequency divisor (both integer and fractional part)
    reg_set(CM_GPDIV[gpclk_number], CM_PASSWD | clock_divisor);              // Set frequency divisor
}

uint32_t readTimerLow(){
    return reg_get(SYSTMR_CLO);
}


/***************
* Main program *
***************/

void microSleep(uint32_t micro){
    uint32_t now, start = readTimerLow();

    do {
        now = readTimerLow();
    } while ( 
        ((now >= start) && (now - start) < micro) ||
        ((now <  start) &&  now < (start + micro))
    );
}

void start_radio(CM_GPCLK gpclk_number, uint32_t carrier_frequency, char* audio_path){

    SF_INFO sfinfo;
    bzero(&sfinfo, sizeof(SF_INFO));
    SNDFILE* inf = sf_open(audio_path, SFM_READ, &sfinfo); // open audio file for reading
    if (!inf) {
        puts("Audio file not found");
        return;
    }
    printf("Audio channels:   %d\n", sfinfo.channels);
    printf("Audio samplerate: %d\n", sfinfo.samplerate);

    float deviation = 75000; // 75 kHz of deviation from nominal carrier (for 200 kHz FM spacing) [https://en.wikipedia.org/wiki/Frequency_deviation]
    float *sample = (float *) malloc(sfinfo.channels * sizeof(float)); // allocate space to store samples
    unsigned int waiting_period = round((double) 1000000.00/sfinfo.samplerate);  // Calculate the waiting time between two consecutive samples in microseconds
    printf("One sample each %d microseconds\n", waiting_period);

    while (sf_readf_float(inf, sample, 1) > 0) { // get ONE sample PER channel

        // convert multi channel audio in mono audio
        float all_sample_sum = 0;
        for(int i = 0; i < sfinfo.channels; i++)
            all_sample_sum += (sample[i] / sfinfo.channels);

        int sample_value = round(all_sample_sum * deviation);  // all_sample_sum contains a number in range [-1.0, +1.0], so the "max" sample_value will be -deviation or +deviation 
        
        set_clock_frequency(gpclk_number, carrier_frequency + sample_value); // Let's modulate!

        microSleep(waiting_period);
    }

    puts("End of file audio");
    set_clock_frequency(gpclk_number, carrier_frequency);
    sf_close(inf);
    free(sample);
}

// Stop clock generator and reset GPIO4 to OUTPUT function
void exit_handler(int signum){
    puts("\nCleaning resources...");
    set_gp_func(4, GP_OUTPUT);        // Set GPIO4 (pin 7) as output
    stop_clock(CM_GPCLK0, CM_PLLD);   // Disable clock generator
    exit(signum);                     // Close process
}

// Set signal handler
void set_signal_handler(int signum, void (*handler)(int)){
    if (signal(signum, handler) == SIG_ERR)         // Set signal handler and check success
        printf("Set signal (%d) error", signum);
}

int main(int argc, char **argv){

    char *audio_path = NULL;
    if (argc < 2){
        puts("Missing audio file");
        exit(-1);
    } else {
        audio_path = argv[1];
    }

    uint32_t carrier_frequency = 105000000;             // Default frequency 105.0 MHz
    if (argc > 2)
        carrier_frequency = atoi(argv[2]);              // Parse first argument as frequency if present

    set_gp_func(4, GP_AF0);                             // Set GPIO4 (pin 7) as clock (Alternative Function 0)
    start_clock(CM_GPCLK0, CM_PLLD, CM_MASH_1S);        // Enable GPCLK0 clock generator (because we are using GPIO4 [pin 7]) using PLLD clock source (500MHz) and 1-stage MASH (to support fractional divisor)
    set_clock_frequency(CM_GPCLK0, carrier_frequency);  // Set the clock frequency of GPCLK0
    printf("Transmitting carrier on %d Hz\n", carrier_frequency);

    set_signal_handler(SIGQUIT, exit_handler);          // Set SIGQUIT signal handler to reset gpio on exit
    set_signal_handler(SIGINT,  exit_handler);          // Set SIGINT  signal handler to reset gpio on exit
    
    start_radio(CM_GPCLK0, carrier_frequency, audio_path);

    exit_handler(0);
    return 0;
}
