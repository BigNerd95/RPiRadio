/*
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
    https://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
        Section 6 General Purpose I/O (GPIO)
        Pages:
            - 90 (Register View)
            - 92 (GPIO Alternate function select register 0)
            - 105 (General Purpose GPIO Clocks, MASH dividers)
            - 107 (Clock Manager General Purpose Clock Control)
            - 108 (Clock Manager General Purpose Clock Divisors)

    https://github.com/raspberrypi/userland/blob/master/host_applications/linux/libs/bcm_host/bcm_host.c
        Functions:
            - unsigned bcm_host_get_peripheral_address(void)
            - unsigned bcm_host_get_peripheral_size(void)
            - unsigned bcm_host_get_sdram_address(void)

    https://pinout.xyz/pinout/pin7_gpio4

License:
    GPL3

Author:
    Lorenzo Santina (BigNerd95)
*/

#include <bcm_host.h>   // Broadcom library to support any version of RaspBerry
#include <stdio.h>      // Input/Output functions
#include <stdint.h>     // Integer types having specified widths
#include <fcntl.h>      // Function open
#include <sys/types.h>  // Sometimes needed by fcntl
#include <sys/stat.h>   // Sometimes needed by fcntl
#include <sys/mman.h>   // Function mmap

// Physical peripheral base addresses
#define BASE_ADDRESS 0x7E000000
// Clock Manager General Purpose Clocks Control registers
#define CM_GP0CTL 0x7E101070    // GPCLK0
#define CM_GP1CTL 0x7E101078    // not used
#define CM_GP2CTL 0x7E101080    // not used

// Clock Manager General Purpose Clock Divisors registers
#define CM_GP0DIV 0x7E101074    // GPCLK0
#define CM_GP1DIV 0x7E10107c    // not used
#define CM_GP2DIV 0x7E101084    // not used

// General Purpose I/O function select registers
#define GPFSEL0  0x7E200000     // BCM pin 0 to 9, we use 4
#define GPFSEL1  0x7E200004     // BCM pin 10 to 19 (register not used)
#define GPFSEL2  0x7E200008     // BCM pin 20 to 29 (register not used)
#define GPFSEL3  0x7E20000C     // BCM pin 30 to 39 (register not used)
#define GPFSEL4  0x7E200010     // BCM pin 40 to 49 (register not used)
#define GPFSEL5  0x7E200014     // BCM pin 50 to 53 (register not used)
#define GPFSEL {GPFSEL0,GPFSEL1,GPFSEL2,GPFSEL3,GPFSEL4,GPFSEL5} // Array structure to access faster to function registers
#define FLAG_PER_REGISTER 10    // There are 10 pin flag per register
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

#define PLLDFREQ     500000000. // PLLD clock source frequency (500MHz)

void* MAP_ADDRESS = NULL;
#define CONVERT(address)    ((unsigned) (address) - BASE_ADDRESS + MAP_ADDRESS)  // convert physical address to virtual address
#define SET(address, value) ((*(uint32_t*) CONVERT(address)) = (value))          // set value to physical address
#define GET(address)        (*(uint32_t*) CONVERT(address))                      // get value from physical address

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

    close(fd);     // Close /dev/mem fd
    return vaddr;  // Return mapped address
}

void map_peripheral(){
    unsigned peripheral_address = bcm_host_get_peripheral_address();  // Broadcom function to get peripheral address (so it support any RaspBerry version)
    unsigned peripheral_size = bcm_host_get_peripheral_size();        // Broadcom function to get peripheral size
    MAP_ADDRESS = memory_map(peripheral_address, peripheral_size);    // Map peripheral memory
}

void check_peripheral_map(){
    if (!MAP_ADDRESS){
        puts("Peripheral not mapped!");
        exit(-1);
    }
}

void set_gp_func(unsigned int pin, GP_FUNCTION function){
    check_peripheral_map();
    if (pin >= 0 && pin <= 53){
        unsigned int reg_number = pin / FLAG_PER_REGISTER;  // calculate register number
        unsigned int pin_number = pin % FLAG_PER_REGISTER;  // calculate pin number of this register

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

void set_clk_frequency(uint32_t frequency){
    check_peripheral_map();
    uint32_t clock_divisor = ((float)(PLLDFREQ / frequency)) * ( 1 << 12 );  // Calculate frequency clock divisor (both integer and fractional part)

    SET(CM_GP0CTL, 0x5A000006);     // Turn off enable flag
    while(GET(CM_GP0CTL) & 0x80);   // Wait for busy flag to turn off

    SET(CM_GP0DIV, 0x5A000000 | clock_divisor);  // Configure frequency divisor
    SET(CM_GP0CTL, 0x5a000206);                  // Source PLLD (500MHz), 1-stage MASH (to support fractional divider)
    SET(CM_GP0CTL, 0x5A000216);                  // Enable clock
    while(!(GET(CM_GP0CTL) & 0x80));             // Wait for busy flag to turn on
}

void exit_handler(int signum){
    check_peripheral_map();
    puts("Cleaning resources...");
    set_gp_func(4, GP_OUTPUT);   // Set GPIO4 as output
    SET(CM_GP0CTL, 0x5A000000);  // Disable the clock generator.
    exit(signum);                // Close program
}

void set_signal_handler(void (*handler)(int)){
    if (signal(SIGQUIT, handler) == SIG_ERR)  // Set SIGQUIT signal handler to reset gpio on exit
        puts("Signal (SIGQUIT) error");
    if (signal(SIGINT, handler) == SIG_ERR)   // Set SIGINT signal handler to reset gpio on exit
        puts("Signal (SIGINT) error");
}

int main(int argc, char **argv){

    uint32_t carrier_frequency = 104500000; // Default frequency 104.5 MHz
    if (argc > 1)
        carrier_frequency = atoi(argv[1]);  // Parse first argument as frequency if present

    // Call this before access peripherals
    map_peripheral();  // Map peripheral with right addresses depending on board type

    set_gp_func(4, GP_AF0);                 // Set pin 4 as clock (Alternative Function 0)
    set_clk_frequency(carrier_frequency);   // Set frequency and enable clock

    set_signal_handler(exit_handler);       // Set handler to clean resources on exit signal
    printf("Transmitting carrier on %d Hz\n", carrier_frequency);
    while(1);                               // Run forever

    return 0;
}
