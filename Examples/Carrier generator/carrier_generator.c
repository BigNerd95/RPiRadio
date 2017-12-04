/*
Goal:
    This example is for educational purposes only, and is fully commented
    This example only generate a carrier on passed frequency or on default frequency 105.0 MHz
    Turn your car radio on and check if there is silence on that frequency

Usage:
    Compile:
        gcc carrier_generator.c -o carrier_generator -L /opt/vc/lib -l bcm_host -I /opt/vc/include
    Run:
        sudo ./carrier_generator
        sudo ./carrier_generator 106200000
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

#include <bcm_host.h>           // Broadcom library to support any version of RaspBerry
#include <stdio.h>              // Input/Output functions
#include <stdint.h>             // Integer types having specified widths
#include <unistd.h>             // Function pause to wait for signals
#include <time.h>               // Function nanosleep
#include <fcntl.h>              // Function open
#include <sys/types.h>          // Sometimes needed by fcntl
#include <sys/stat.h>           // Sometimes needed by fcntl
#include <sys/mman.h>           // Function mmap



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


// Address conversion macro

void* VIRTUAL_BASE_ADDRESS = NULL;       // Global variable where peripherals virtual base address will be contained after mapping physical address
#define CONVERT(address)    ((unsigned) (address) - BUS_BASE_ADDRESS + VIRTUAL_BASE_ADDRESS)  // convert BUS address to virtual address
#define SET(address, value) ((*(uint32_t*) CONVERT(address)) = (value))          // set value to BUS address
#define GET(address)        (*(uint32_t*) CONVERT(address))                      // get value from BUS address



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
    VIRTUAL_BASE_ADDRESS = map_memory(peripheral_address, peripheral_size);    // Map peripheral memory
}

// Map peripherals if they are not mapped
void check_peripheral(){
    if (!VIRTUAL_BASE_ADDRESS)
        map_peripheral();
}

// Set the value of the given BUS addres
void reg_set(unsigned address, uint32_t value){
    check_peripheral();             // Check if peripherals are mapped
    SET(address, value);            // Set the value
}

// Get the value of the given BUS address
uint32_t reg_get(unsigned address){
    check_peripheral();             // Check if peripherals are mapped
    return GET(address);            // Get the value
}

// Set gpio pin function (pin number is in BCM format)
void set_gp_func(unsigned int pin, GP_FUNCTION function){
    if (pin >= 0 && pin <= 53){
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



/***************
* Main program *
***************/

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

// Set handler to clean resources on exit signals
void run_forever(){
    set_signal_handler(SIGQUIT, exit_handler);      // Set SIGQUIT (ctrl-\) signal handler to reset gpio on exit
    set_signal_handler(SIGINT,  exit_handler);      // Set SIGINT  (ctrl-c) signal handler to reset gpio on exit
    pause();                                        // Wait for signals (unistd function), blocked waiting (so the CPU is not overloaded)
}

int main(int argc, char **argv){

    uint32_t carrier_frequency = 105000000;             // Default frequency 105.0 MHz
    if (argc > 1)
        carrier_frequency = atoi(argv[1]);              // Parse first argument as frequency if present

    set_gp_func(4, GP_AF0);                             // Set GPIO4 (pin 7) as clock (Alternative Function 0)
    start_clock(CM_GPCLK0, CM_PLLD, CM_MASH_1S);        // Enable GPCLK0 clock generator (because we are using GPIO4 [pin 7]) using PLLD clock source (500MHz) and 1-stage MASH (to support fractional divisor)
    set_clock_frequency(CM_GPCLK0, carrier_frequency);  // Set the clock frequency of GPCLK0
    printf("Transmitting carrier on %d Hz\n", carrier_frequency);

    run_forever();                                      // Keep this process alive
    return 0;
}
