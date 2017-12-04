#!/usr/bin/env python3

import struct, mmap
import sys, signal


class Peripheral():

    # Peripheral BUS base address
    BUS_BASE_ADDRESS = 0x7E000000


    # GPIO

    # There are 10 pin flags per register
    FLAGS_PER_REGISTER = 10

    # General Purpose I/O function select registers
    GPFSEL = [
        0x7E200000, # GPFSEL0 BCM pin  0 to  9 (we use 4 [GPIO4 pin 7])
        0x7E200004, # GPFSEL1 BCM pin 10 to 19 (not used in this example)
        0x7E200008, # GPFSEL2 BCM pin 20 to 29 (not used in this example)
        0x7E20000C, # GPFSEL3 BCM pin 30 to 39 (not used in this example)
        0x7E200010, # GPFSEL4 BCM pin 40 to 49 (not used in this example)
        0x7E200014  # GPFSEL5 BCM pin 50 to 53 (not used in this example)
    ]

    # GPIO functions flags (each flag is 3 bit long)
    GP_INPUT  = 0b000
    GP_OUTPUT = 0b001
    GP_AF0    = 0b100
    GP_AF1    = 0b101
    GP_AF2    = 0b110
    GP_AF3    = 0b111
    GP_AF4    = 0b011
    GP_AF5    = 0b010


    # CLOCK
    
    # Clock Manager General Purpose Clocks Control registers
    CM_GPCTL = [
        0x7E101070, # CM_GP0CTL (GPCLK0) Control register [Alternative Function 0 (GP_AF0) of GPIO4 (pin  7)]
        0x7E101078, # CM_GP1CTL (GPCLK1) Control register [Alternative Function 0 (GP_AF0) of GPIO5 (pin 29)] (not used in this example)
        0x7E101080  # CM_GP2CTL (GPCLK2) Control register [Alternative Function 0 (GP_AF0) of GPIO6 (pin 31)] (not used in this example)
    ]

    # Clock Manager General Purpose Clock Divisors registers
    CM_GPDIV = [
        0x7E101074, # CM_GP0DIV (GPCLK0) Divisor register [Alternative Function 0 (GP_AF0) of GPIO4 (pin  7)]
        0x7E10107C, # CM_GP1DIV (GPCLK1) Divisor register [Alternative Function 0 (GP_AF0) of GPIO5 (pin 29)] (not used in this example)
        0x7E101084  # CM_GP2DIV (GPCLK2) Divisor register [Alternative Function 0 (GP_AF0) of GPIO6 (pin 31)] (not used in this example)
    ]

    # Clock Managers General Purpose Clocks
    CM_GPCLK0 = 0
    CM_GPCLK1 = 1
    CM_GPCLK2 = 2

    # Clock Manager flags
    CM_PASSWD = (0x5A << 24)
    CM_ENAB   = (1 << 4)
    CM_BUSY   = (1 << 7)

    # Clock Manager Control register mash flags
    CM_MASH_INT = (0 << 9)
    CM_MASH_1S  = (1 << 9)
    CM_MASH_2S  = (2 << 9)
    CM_MASH_3S  = (3 << 9)

    # Clock Manager Control register SOURCE flags
    CM_GND        = 0
    CM_OSCILLATOR = 1
    CM_TESTDEBUD0 = 2
    CM_TESTDEBUG1 = 3
    CM_PLLA       = 4
    CM_PLLC       = 5
    CM_PLLD       = 6
    CM_HDMI       = 7

    # PLLD clock source frequency (500MHz)
    PLLD_FREQ = 500000000


    def __init__(self):
        peripheral_address = Peripheral.__bcm_host_get_peripheral_address__()
        peripheral_size    = Peripheral.__bcm_host_get_peripheral_size__()
        self.PERIPHERAL_MEMORY = Peripheral.__map_peripheral_memory__(peripheral_address, peripheral_size)

    def __get_dt_ranges__(filename, offset):
        address = 0xFFFFFFFF
        with open(filename, "rb") as fp:
            fp.seek(offset, 0)
            buf = fp.read(4)
            if len(buf) == 4:
                address = struct.unpack('>I', buf)[0]
        return address

    def __bcm_host_get_peripheral_address__():
        address = Peripheral.__get_dt_ranges__("/proc/device-tree/soc/ranges", 4)
        if address == 0xFFFFFFFF:
            address = 0x20000000
        return address


    def __bcm_host_get_peripheral_size__():
        address = Peripheral.__get_dt_ranges__("/proc/device-tree/soc/ranges", 8)
        if address == 0xFFFFFFFF:
            address = 0x01000000
        return address

    def __map_peripheral_memory__(address, size):
        with open("/dev/mem", "r+b") as fp:
            return mmap.mmap(fp.fileno(), size, offset=address)

    # Convert BUS address to virtual address
    def __convert_address__(address):
        return address - Peripheral.BUS_BASE_ADDRESS

    # Set the value of the given BUS addres
    def __reg_set__(self, address, value):
        relative_address = Peripheral.__convert_address__(address)
        self.PERIPHERAL_MEMORY[relative_address : relative_address + 4] = struct.pack("<I", value)

    # Get the value of the given BUS address
    def __reg_get__(self, address):
        relative_address = Peripheral.__convert_address__(address)
        return struct.unpack("<I", self.PERIPHERAL_MEMORY[relative_address : relative_address + 4])[0]

    def print_reg(self, reg):
        print(hex(self.__reg_get__(reg)))

    # Set gpio pin function (pin number is in BCM format)
    def set_gp_func(self, pin, function):
        if pin in range(0, 54):
            reg_number = int(pin / Peripheral.FLAGS_PER_REGISTER)   # Calculate register number
            pin_number =     pin % Peripheral.FLAGS_PER_REGISTER    # Calculate pin number of this register

            value = self.__reg_get__(Peripheral.GPFSEL[reg_number]) # Get GPFSELn register value
            value &= ~(   0b111 << 3 * pin_number)                  # Zeroes out the 3 bits relative to this pin
            value |=  (function << 3 * pin_number)                  # Set pin function
            self.__reg_set__(Peripheral.GPFSEL[reg_number], value)  # Set GPFSELn register new value
        else:
            print("Invalid pin number!")

    def set_clock_frequency(self, gpclk_number, frequency):
        clock_divisor = (Peripheral.PLLD_FREQ / frequency) * (1 << 12)   # Calculate clock frequency divisor (both integer and fractional part)
        self.__reg_set__(Peripheral.CM_GPDIV[gpclk_number], Peripheral.CM_PASSWD | int(clock_divisor))      # Set frequency divisor

    # Set frequency divisor, source, mash and enable clock generator
    def start_clock(self, gpclk_number, clk_source, mash_stage):
        self.stop_clock(gpclk_number, clk_source)
        self.__reg_set__(Peripheral.CM_GPCTL[gpclk_number], Peripheral.CM_PASSWD | mash_stage | clk_source) # Set source and MASH
        
        while not self.__reg_get__(Peripheral.CM_GPCTL[gpclk_number]) & Peripheral.CM_BUSY:                 # Unless busy flag turns on
            self.__reg_set__(Peripheral.CM_GPCTL[gpclk_number], Peripheral.CM_PASSWD | self.__reg_get__(Peripheral.CM_GPCTL[gpclk_number]) | Peripheral.CM_ENAB) # Enable clock generator
    
    # Disable clock generator and wait until busy flag turns off
    def stop_clock(self, gpclk_number, clk_source):
        while self.__reg_get__(Peripheral.CM_GPCTL[gpclk_number]) & Peripheral.CM_BUSY:            # Unless busy flag turns off
            self.__reg_set__(Peripheral.CM_GPCTL[gpclk_number], Peripheral.CM_PASSWD | clk_source) # Disable clock generator




# Stop clock generator and reset GPIO4 to OUTPUT function
def exit_handler(signum, frame):
    print("Cleaning resources...")
    per.set_gp_func(4, Peripheral.GP_OUTPUT)                 # Set GPIO4 (pin 7) as output
    per.stop_clock(Peripheral.CM_GPCLK0, Peripheral.CM_PLLD) # Disable clock generator
    sys.exit(signum)                                         # Close process

# Set handler to clean resources on exit signals
def run_forever():
    signal.signal(signal.SIGQUIT, exit_handler)      # Set SIGQUIT (ctrl-\) signal handler to reset gpio on exit
    signal.signal(signal.SIGINT,  exit_handler)      # Set SIGINT  (ctrl-c) signal handler to reset gpio on exit
    signal.pause()                                   # Wait for signals (blocked waiting, so the CPU is not overloaded)


if __name__ == "__main__":

    carrier_frequency = 105000000
    if len(sys.argv) > 1:
        carrier_frequency = int(sys.argv[1])

    per = Peripheral()
    per.set_gp_func(4, Peripheral.GP_AF0)
    per.start_clock(Peripheral.CM_GPCLK0, Peripheral.CM_PLLD, Peripheral.CM_MASH_1S)
    per.set_clock_frequency(Peripheral.CM_GPCLK0, carrier_frequency)

    print("Transmitting carrier on " + str(carrier_frequency) + " Hz")

    run_forever()
