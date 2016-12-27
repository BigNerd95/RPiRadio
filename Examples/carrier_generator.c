/*
Goal:
    This example is for educational purposes only, and is fully commented
    This example only generate a carrier on passed frequency or on default frequency 104.5 MHz
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
            - 90 (Register View)
            - 92 (GPIO Alternate function select register 0)
            - 105 (General Purpose GPIO Clocks, MASH dividers)
            - 107 (Clock Manager General Purpose Clock Control)
            - 108 (Clock Manager General Purpose Clock Divisors)

    https://github.com/raspberrypi/userland/blob/master/host_applications/linux/libs/bcm_host/bcm_host.c
        Functions to allow compatibility on RPi 1 and successors:
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
#include <fcntl.h>              // Function open
#include <sys/types.h>          // Sometimes needed by fcntl
#include <sys/stat.h>           // Sometimes needed by fcntl
#include <sys/mman.h>           // Function mmap


// Physical peripheral base addresses
#define BASE_ADDRESS 0x7E000000


// General Purpose I/O function select registers
#define GPFSEL0  0x7E200000     // BCM pin 0 to 9, we use 4 [GPIO4 pin 7]
#define GPFSEL1  0x7E200004     // BCM pin 10 to 19 (register not used in this example)
#define GPFSEL2  0x7E200008     // BCM pin 20 to 29 (register not used in this example)
#define GPFSEL3  0x7E20000C     // BCM pin 30 to 39 (register not used in this example)
#define GPFSEL4  0x7E200010     // BCM pin 40 to 49 (register not used in this example)
#define GPFSEL5  0x7E200014     // BCM pin 50 to 53 (register not used in this example)
#define GPFSEL {GPFSEL0,GPFSEL1,GPFSEL2,GPFSEL3,GPFSEL4,GPFSEL5} // Array structure to access faster to function registers
#define FLAGS_PER_REGISTER 10    // There are 10 pin flags per register
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


// Clock Manager General Purpose Clocks Control registers
#define CM_GP0CTL 0x7E101070    // GPCLK0 Control register [GPIO4 pin  7]
#define CM_GP1CTL 0x7E101078    // GPCLK1 Control register [GPIO5 pin 29] (not used in this example)
#define CM_GP2CTL 0x7E101080    // GPCLK2 Control register [GPIO6 pin 31] (not used in this example)
#define CM_CTL {CM_GP0CTL,CM_GP1CTL,CM_GP2CTL}
// Clock Manager General Purpose Clock Divisors registers
#define CM_GP0DIV 0x7E101074    // GPCLK0 Divisor register [GPIO4 pin  7]
#define CM_GP1DIV 0x7E10107C    // GPCLK1 Divisor register [GPIO5 pin 29] (not used in this example)
#define CM_GP2DIV 0x7E101084    // GPCLK2 Divisor register [GPIO6 pin 31] (not used in this example)
#define CM_DIV {CM_GP0DIV,CM_GP1DIV,CM_GP2DIV}
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
typedef enum {
    CM_MASH_INT = (0 << 9),
    CM_MASH_1S  = (1 << 9),
    CM_MASH_2S  = (2 << 9),
    CM_MASH_3S  = (3 << 9)
} CM_MASH;
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

#define PLLDFREQ     500000000. // PLLD clock source frequency (500MHz)


void* MAP_ADDRESS = NULL;       // Global variable where peripherals map address will be contained
#define CONVERT(address)    ((unsigned) (address) - BASE_ADDRESS + MAP_ADDRESS)  // convert physical address to virtual address
#define SET(address, value) ((*(uint32_t*) CONVERT(address)) = (value))          // set value to physical address
#define GET(address)        (*(uint32_t*) CONVERT(address))                      // get value from physical address

// Maps 'address' in the virtual address space of the process
void* memory_map(unsigned address, size_t size) {

    int fd = open("/dev/mem", O_RDWR | O_SYNC);     // Open /dev/mem to map 'address'
    if (fd < 0){                                    // Check if open of /dev/mem is failed
        puts("Can't open /dev/mem\nRun as root!");
        exit(-1);
    }

    void* vaddr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, address);  // map 'address' from /dev/mem fd with size of 'size'
    if (vaddr == MAP_FAILED){                       // Check if mmap is failed
        puts("Failed to map memory");
        exit(-1);
    }

    close(fd);                                      // Close /dev/mem fd
    return vaddr;                                   // Return mapped address
}

// Maps the correct peripheral address in the virtual address space of the process
void map_peripheral(){
    unsigned peripheral_address = bcm_host_get_peripheral_address();  // Broadcom function to get peripheral address (so it support any Raspberry Pi version)
    unsigned peripheral_size = bcm_host_get_peripheral_size();        // Broadcom function to get peripheral size
    MAP_ADDRESS = memory_map(peripheral_address, peripheral_size);    // Map peripheral memory
}

// Check if peripherals are mapped
void check_peripheral_map(){
    if (!MAP_ADDRESS){
        puts("Peripheral not mapped!");
        exit(-1);
    }
}

// Set gpio pin function (pin number is in BCM format)
void set_gp_func(unsigned int pin, GP_FUNCTION function){
    check_peripheral_map();                     // Always check if peripherals are mapped before access to them
    if (pin >= 0 && pin <= 53){
        unsigned int reg_number = pin / FLAGS_PER_REGISTER;  // calculate register number
        unsigned int pin_number = pin % FLAGS_PER_REGISTER;  // calculate pin number of this register

        unsigned reg[] = GPFSEL;                 // init array of pointer to GPFSEL registers addresses
        uint32_t value = GET(reg[reg_number]);   // get register value
        value &= ~(   0b111 << 3 * pin_number);  // zeroes out the 3 bits of this pin
        value |=  (function << 3 * pin_number);  // set pin function
        SET(reg[reg_number], value);             // set new register value
    } else {
        puts("Invalid pin number!");
        exit(-1);
    }
}

void stop_clk_generator(CM_GPCLK gpclk_number, CM_SRC clk_source){
    check_peripheral_map();                                     // Always check if peripherals are mapped before access to them
    unsigned reg_ctl[] = CM_CTL;                                // Init array of pointer to CM_CTL registers addresses

    while(GET(reg_ctl[gpclk_number]) & CM_BUSY){                // Unless busy flag turn off
        SET(reg_ctl[gpclk_number], CM_PASSWD | clk_source);     // Disable clock generator
    }
}

void start_clk_generator(CM_GPCLK gpclk_number, uint32_t clock_divisor, CM_SRC clk_source, CM_MASH mash_stage){
    check_peripheral_map();                                             // Always check if peripherals are mapped before access to them
    unsigned reg_ctl[] = CM_CTL;                                        // Init array of pointer to CM_CTL registers addresses
    unsigned reg_div[] = CM_DIV;                                        // Init array of pointer to CM_DIV registers addresses

    stop_clk_generator(gpclk_number, clk_source);                      // Disable clock generator before any changes

    SET(reg_div[gpclk_number], CM_PASSWD | clock_divisor);              // Set frequency divisor
    SET(reg_ctl[gpclk_number], CM_PASSWD | mash_stage | clk_source);    // Set source and MASH

    while(!(GET(reg_ctl[gpclk_number]) & CM_BUSY)){                     // Unless busy flag turn on
        SET(reg_ctl[gpclk_number], CM_PASSWD | GET(reg_ctl[gpclk_number]) | CM_ENAB);  // Enable clock generator
    }
}

// Set clock frequency to GPCLK pin
void set_clk_frequency(CM_GPCLK gpclk_number, uint32_t frequency){
    uint32_t clock_divisor = ((float)(PLLDFREQ / frequency)) * (1 << 12);   // Calculate frequency clock divisor (both integer and fractional part)
    start_clk_generator(gpclk_number, clock_divisor, CM_PLLD, CM_MASH_1S);  // Set clock_divisor, source PLLD (500MHz), 1-stage MASH (to support fractional divisor)
}

// Stop clock generator and reset GPIO4 to OUTPUT function
void exit_handler(int signum){
    puts("Cleaning resources...");
    set_gp_func(4, GP_OUTPUT);                  // Set GPIO4 as output
    stop_clk_generator(CM_GPCLK0, CM_PLLD);    // Disable clock generator
    exit(signum);                               // Close program
}

// Set exit program handler
void set_signal_handler(int signum, void (*handler)(int)){
    if (signal(signum, handler) == SIG_ERR)         // Set handler to passed signal
        printf("Set signal (%d) error", signum);
}

// Set handler to clean resources on exit signal
void run_forever(){
    set_signal_handler(SIGQUIT, exit_handler);      // Set SIGQUIT signal handler to reset gpio on exit
    set_signal_handler(SIGINT,  exit_handler);      // Set SIGINT  signal handler to reset gpio on exit
    pause();                                        // Wait for signals
}

int main(int argc, char **argv){

    uint32_t carrier_frequency = 104500000;        // Default frequency 104.5 MHz
    if (argc > 1)
        carrier_frequency = atoi(argv[1]);         // Parse first argument as frequency if present

    // Call this only once, before access peripherals
    map_peripheral();                              // Map peripheral with right addresses depending on board type

    set_gp_func(4, GP_AF0);                        // Set GPIO4 [pin 7] as clock (Alternative Function 0)
    set_clk_frequency(CM_GPCLK0, carrier_frequency);  // Set the clock frequency to GPCLK0 (because we are using GPIO4 [pin 7])

    printf("Transmitting carrier on %d Hz\n", carrier_frequency);
    run_forever();                                 // Run forever

    return 0;
}
