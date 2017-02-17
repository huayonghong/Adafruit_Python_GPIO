# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import atexit
import logging
import math
import os
import subprocess
import sys
import time
import struct

import ftdi1 as ftdi

import Adafruit_GPIO.GPIO as GPIO


logger = logging.getLogger(__name__)

FT232H_VID = 0x0403   # Default FTDI FT232H vendor ID
FT232H_PID = 0x6014   # Default FTDI FT232H product ID

MSBFIRST = 0
LSBFIRST = 1

_REPEAT_DELAY = 4


def _check_running_as_root():
    # NOTE: Checking for root with user ID 0 isn't very portable, perhaps
    # there's a better alternative?
    if os.geteuid() != 0:
        raise RuntimeError('Expected to be run by root user! Try running with sudo.')

def disable_FTDI_driver():
    """Disable the FTDI drivers for the current platform.  This is necessary
    because they will conflict with libftdi and accessing the FT232H.  Note you
    can enable the FTDI drivers again by calling enable_FTDI_driver.
    """
    logger.debug('Disabling FTDI driver.')
    if sys.platform == 'darwin':
        logger.debug('Detected Mac OSX')
        # Mac OS commands to disable FTDI driver.
        _check_running_as_root()
        subprocess.call('kextunload -b com.apple.driver.AppleUSBFTDI', shell=True)
        subprocess.call('kextunload /System/Library/Extensions/FTDIUSBSerialDriver.kext', shell=True)
    elif sys.platform.startswith('linux'):
        logger.debug('Detected Linux')
        # Linux commands to disable FTDI driver.
        _check_running_as_root()
        subprocess.call('modprobe -r -q ftdi_sio', shell=True)
        subprocess.call('modprobe -r -q usbserial', shell=True)
    # Note there is no need to disable FTDI drivers on Windows!

def enable_FTDI_driver():
    """Re-enable the FTDI drivers for the current platform."""
    logger.debug('Enabling FTDI driver.')
    if sys.platform == 'darwin':
        logger.debug('Detected Mac OSX')
        # Mac OS commands to enable FTDI driver.
        _check_running_as_root()
        subprocess.check_call('kextload -b com.apple.driver.AppleUSBFTDI', shell=True)
        subprocess.check_call('kextload /System/Library/Extensions/FTDIUSBSerialDriver.kext', shell=True)
    elif sys.platform.startswith('linux'):
        logger.debug('Detected Linux')
        # Linux commands to enable FTDI driver.
        _check_running_as_root()
        subprocess.check_call('modprobe -q ftdi_sio', shell=True)
        subprocess.check_call('modprobe -q usbserial', shell=True)

def use_FT232H():
    """Disable any built in FTDI drivers which will conflict and cause problems
    with libftdi (which is used to communicate with the FT232H).  Will register
    an exit function so the drivers are re-enabled on program exit.
    """
    disable_FTDI_driver()
    atexit.register(enable_FTDI_driver)

def enumerate_device_serials(vid=FT232H_VID, pid=FT232H_PID):
    """Return a list of all FT232H device serial numbers connected to the
    machine.  You can use these serial numbers to open a specific FT232H device
    by passing it to the FT232H initializer's serial parameter.
    """
    try:
        # Create a libftdi context.
        ctx = None
        ctx = ftdi.new()
        # Enumerate FTDI devices.
        device_list = None
        count, device_list = ftdi.usb_find_all(ctx, vid, pid)
        if count < 0:
            raise RuntimeError('ftdi_usb_find_all returned error {0}: {1}'.format(count, ftdi.get_error_string(self._ctx)))
        # Walk through list of devices and assemble list of serial numbers.
        devices = []
        while device_list is not None:
            # Get USB device strings and add serial to list of devices.
            ret, manufacturer, description, serial = ftdi.usb_get_strings(ctx, device_list.dev, 256, 256, 256)
            if ret < 0:
                raise RuntimeError('ftdi_usb_get_strings returned error {0}: {1}'.format(ret, ftdi.get_error_string(self._ctx)))
            devices.append(serial)
            device_list = device_list.next
        return devices
    finally:
        # Make sure to clean up list and context when done.
        if device_list is not None:
            ftdi.list_free(device_list)
        if ctx is not None:
            ftdi.free(ctx)


class FT232H(GPIO.BaseGPIO):
    # Make GPIO constants that match main GPIO class for compatibility.
    HIGH = GPIO.HIGH
    LOW  = GPIO.LOW
    IN   = GPIO.IN
    OUT  = GPIO.OUT

    def __init__(self, vid=FT232H_VID, pid=FT232H_PID, serial=None):
        """Create a FT232H object.  Will search for the first available FT232H
        device with the specified USB vendor ID and product ID (defaults to
        FT232H default VID & PID).  Can also specify an optional serial number
        string to open an explicit FT232H device given its serial number.  See
        the FT232H.enumerate_device_serials() function to see how to list all
        connected device serial numbers.
        """
        # Initialize FTDI device connection.
        self._ctx = ftdi.new()
        if self._ctx == 0:
            raise RuntimeError('ftdi_new failed! Is libftdi1 installed?')
        # Register handler to close and cleanup FTDI context on program exit.
        atexit.register(self.close)
        if serial is None:
            # Open USB connection for specified VID and PID if no serial is specified.
            self._check(ftdi.usb_open, vid, pid)
        else:
            # Open USB connection for VID, PID, serial.
            self._check(ftdi.usb_open_string, 's:{0}:{1}:{2}'.format(vid, pid, serial))
        # Reset device.
        self._check(ftdi.usb_reset)
        # Disable flow control. Commented out because it is unclear if this is necessary.
        #self._check(ftdi.setflowctrl, ftdi.SIO_DISABLE_FLOW_CTRL)
        # Change read & write buffers to maximum size, 65535 bytes.
        self._check(ftdi.read_data_set_chunksize, 65535)
        self._check(ftdi.write_data_set_chunksize, 65535)
        # Clear pending read data & write buffers.
        self._check(ftdi.usb_purge_buffers)
        # Enable MPSSE and syncronize communication with device.
        self._mpsse_enable()
        self._mpsse_sync()
        # Initialize all GPIO as inputs.
        self._write('\x80\x00\x00\x82\x00\x00')
        self._direction = 0x0000
        self._level = 0x0000

    def close(self):
        """Close the FTDI device.  Will be automatically called when the program ends."""
        if self._ctx is not None:
            ftdi.free(self._ctx)
        self._ctx = None

    def _write(self, string):
        """Helper function to call write_data on the provided FTDI device and
        verify it succeeds.
        """
        # Get modem status. Useful to enable for debugging.
        #ret, status = ftdi.poll_modem_status(self._ctx)
        #if ret == 0:
        #	logger.debug('Modem status {0:02X}'.format(status))
        #else:
        #	logger.debug('Modem status error {0}'.format(ret))
        length = len(string)
        ret = ftdi.write_data(self._ctx, string, length)
        # Log the string that was written in a python hex string format using a very
        # ugly one-liner list comprehension for brevity.
        #logger.debug('Wrote {0}'.format(''.join(['\\x{0:02X}'.format(ord(x)) for x in string])))
        if ret < 0:
            raise RuntimeError('ftdi_write_data failed with error {0}: {1}'.format(ret, ftdi.get_error_string(self._ctx)))
        if ret != length:
            raise RuntimeError('ftdi_write_data expected to write {0} bytes but actually wrote {1}!'.format(length, ret))

    def _check(self, command, *args):
        """Helper function to call the provided command on the FTDI device and
        verify the response matches the expected value.
        """
        ret = command(self._ctx, *args)
        logger.debug('Called ftdi_{0} and got response {1}.'.format(command.__name__, ret))
        if ret != 0:
            raise RuntimeError('ftdi_{0} failed with error {1}: {2}'.format(command.__name__, ret, ftdi.get_error_string(self._ctx)))

    def _poll_read(self, expected, timeout_s=5.0):
        """Helper function to continuously poll reads on the FTDI device until an
        expected number of bytes are returned.  Will throw a timeout error if no
        data is received within the specified number of timeout seconds.  Returns
        the read data as a string if successful, otherwise raises an execption.
        """
        start = time.time()
        # Start with an empty response buffer.
        response = bytearray(expected)
        index = 0
        # Loop calling read until the response buffer is full or a timeout occurs.
        while time.time() - start <= timeout_s:
            ret, data = ftdi.read_data(self._ctx, expected - index)
            # Fail if there was an error reading data.
            if ret < 0:
                raise RuntimeError('ftdi_read_data failed with error code {0}.'.format(ret))
            # Add returned data to the buffer.
            response[index:index+ret] = data[:ret]
            index += ret
            # Buffer is full, return the result data.
            if index >= expected:
                return str(response)
            time.sleep(0.01)
        raise RuntimeError('Timeout while polling ftdi_read_data for {0} bytes!'.format(expected))

    def _mpsse_enable(self):
        """Enable MPSSE mode on the FTDI device."""
        # Reset MPSSE by sending mask = 0 and mode = 0
        self._check(ftdi.set_bitmode, 0, 0)
        # Enable MPSSE by sending mask = 0 and mode = 2
        self._check(ftdi.set_bitmode, 0, 2)

    def _mpsse_sync(self, max_retries=10):
        """Synchronize buffers with MPSSE by sending bad opcode and reading expected
        error response.  Should be called once after enabling MPSSE."""
        # Send a bad/unknown command (0xAB), then read buffer until bad command
        # response is found.
        self._write('\xAB')
        # Keep reading until bad command response (0xFA 0xAB) is returned.
        # Fail if too many read attempts are made to prevent sticking in a loop.
        tries = 0
        sync = False
        while not sync:
            data = self._poll_read(2)
            if data == '\xFA\xAB':
                sync = True
            tries += 1
            if tries >= max_retries:
                raise RuntimeError('Could not synchronize with FT232H!')

    def mpsse_clock(self, clock_hz, three_phase=False):
        # Compute divisor for requested clock.
        # Use equation from section 3.8.1 of:
        #  http://www.ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
        # Note equation is using 60mhz master clock instead of 12mhz.
        divisor = int(math.ceil((30000000.0-float(clock_hz))/float(clock_hz))) & 0xFFFF
        if three_phase:
            divisor = int(divisor*(2.0/3.0))
        logger.debug('Setting clockspeed with divisor value {0}'.format(divisor))
        # return command to set divisor from low and high byte values.
        return bytearray((0x86, divisor & 0xFF, (divisor >> 8) & 0xFF))

    def mpsse_set_clock(self, clock_hz, adaptive=False, three_phase=False):
        """Set the clock speed of the MPSSE engine.  Can be any value from 450hz
        to 30mhz and will pick that speed or the closest speed below it.
        """
        # Disable clock divisor by 5 to enable faster speeds on FT232H.
        self._write('\x8A')
        # Turn on/off adaptive clocking.
        if adaptive:
            self._write('\x96')
        else:
            self._write('\x97')
        # Turn on/off three phase clock (needed for I2C).
        # Also adjust the frequency for three-phase clocking as specified in section 2.2.4
        # of this document:
        #   http://www.ftdichip.com/Support/Documents/AppNotes/AN_255_USB%20to%20I2C%20Example%20using%20the%20FT232H%20and%20FT201X%20devices.pdf
        if three_phase:
            self._write('\x8C')
        else:
            self._write('\x8D')
        clock_command = self.mpsse_clock(clock_hz, three_phase)
        self._write(str(clock_command))

    def mpsse_read_gpio(self):
        """Read both GPIO bus states and return a 16 bit value with their state.
        D0-D7 are the lower 8 bits and C0-C7 are the upper 8 bits.
        """
        # Send command to read low byte and high byte.
        self._write('\x81\x83')
        # Wait for 2 byte response.
        data = self._poll_read(2)
        # Assemble response into 16 bit value.
        low_byte = ord(data[0])
        high_byte = ord(data[1])
        logger.debug('Read MPSSE GPIO low byte = {0:02X} and high byte = {1:02X}'.format(low_byte, high_byte))
        return (high_byte << 8) | low_byte

    def mpsse_gpio(self):
        """Return command to update the MPSSE GPIO state to the current direction
        and level.
        """
        level_low  = chr(self._level & 0xFF)
        level_high = chr((self._level >> 8) & 0xFF)
        dir_low  = chr(self._direction & 0xFF)
        dir_high = chr((self._direction >> 8) & 0xFF)
        return str(bytearray((0x80, level_low, dir_low, 0x82, level_high, dir_high)))

    def mpsse_write_gpio(self):
        """Write the current MPSSE GPIO state to the FT232H chip."""
        self._write(self.mpsse_gpio())

    def get_i2c_device(self, address, **kwargs):
        """Return an I2CDevice instance using this FT232H object and the provided
        I2C address.  Meant to be passed as the i2c_provider parameter to objects
        which use the Adafruit_Python_GPIO library for I2C.
        """
        return I2CDevice(self, address, **kwargs)

    # GPIO functions below:

    def _setup_pin(self, pin, mode):
        if pin < 0 or pin > 15:
            raise ValueError('Pin must be between 0 and 15 (inclusive).')
        if mode not in (GPIO.IN, GPIO.OUT):
            raise ValueError('Mode must be GPIO.IN or GPIO.OUT.')
        if mode == GPIO.IN:
            # Set the direction and level of the pin to 0.
            self._direction &= ~(1 << pin) & 0xFFFF
            self._level     &= ~(1 << pin) & 0xFFFF
        else:
            # Set the direction of the pin to 1.
            self._direction |= (1 << pin) & 0xFFFF

    def setup(self, pin, mode):
        """Set the input or output mode for a specified pin.  Mode should be
        either OUT or IN."""
        self._setup_pin(pin, mode)
        self.mpsse_write_gpio()

    def setup_pins(self, pins, values={}, write=True):
        """Setup multiple pins as inputs or outputs at once.  Pins should be a
        dict of pin name to pin mode (IN or OUT).  Optional starting values of
        pins can be provided in the values dict (with pin name to pin value).
        """
        # General implementation that can be improved by subclasses.
        for pin, mode in iter(pins.items()):
            self._setup_pin(pin, mode)
        for pin, value in iter(values.items()):
            self._output_pin(pin, value)
        if write:
            self.mpsse_write_gpio()

    def _output_pin(self, pin, value):
        if value:
            self._level |= (1 << pin) & 0xFFFF
        else:
            self._level &= ~(1 << pin) & 0xFFFF

    def output(self, pin, value):
        """Set the specified pin the provided high/low value.  Value should be
        either HIGH/LOW or a boolean (true = high)."""
        if pin < 0 or pin > 15:
            raise ValueError('Pin must be between 0 and 15 (inclusive).')
        self._output_pin(pin, value)
        self.mpsse_write_gpio()

    def output_pins(self, pins, write=True):
        """Set multiple pins high or low at once.  Pins should be a dict of pin
        name to pin value (HIGH/True for 1, LOW/False for 0).  All provided pins
        will be set to the given values.
        """
        for pin, value in iter(pins.items()):
            self._output_pin(pin, value)
        if write:
            self.mpsse_write_gpio()

    def input(self, pin):
        """Read the specified pin and return HIGH/true if the pin is pulled high,
        or LOW/false if pulled low."""
        return self.input_pins([pin])[0]

    def input_pins(self, pins):
        """Read multiple pins specified in the given list and return list of pin values
        GPIO.HIGH/True if the pin is pulled high, or GPIO.LOW/False if pulled low."""
        if [pin for pin in pins if pin < 0 or pin > 15]:
            raise ValueError('Pin must be between 0 and 15 (inclusive).')
        _pins = self.mpsse_read_gpio()
        return [((_pins >> pin) & 0x0001) == 1 for pin in pins]


class SPI(object):
    def __init__(self, ft232h, cs=None, max_speed_hz=1000000, mode=0, bitorder=MSBFIRST):
        self._ft232h = ft232h
        # Initialize chip select pin if provided to output high.
        if cs is not None:
            ft232h.setup(cs, GPIO.OUT)
            ft232h.set_high(cs)
        self._cs = cs
        # Initialize clock, mode, and bit order.
        self.set_clock_hz(max_speed_hz)
        self.set_mode(mode)
        self.set_bit_order(bitorder)

    def _assert_cs(self):
        if self._cs is not None:
            self._ft232h.set_low(self._cs)

    def _deassert_cs(self):
        if self._cs is not None:
            self._ft232h.set_high(self._cs)

    def set_clock_hz(self, hz):
        """Set the speed of the SPI clock in hertz.  Note that not all speeds
        are supported and a lower speed might be chosen by the hardware.
        """
        self._ft232h.mpsse_set_clock(hz)

    def set_mode(self, mode):
        """Set SPI mode which controls clock polarity and phase.  Should be a
        numeric value 0, 1, 2, or 3.  See wikipedia page for details on meaning:
        http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
        """
        if mode < 0 or mode > 3:
            raise ValueError('Mode must be a value 0, 1, 2, or 3.')
        if mode == 0:
            # Mode 0 captures on rising clock, propagates on falling clock
            self.write_clock_ve = 1
            self.read_clock_ve  = 0
            # Clock base is low.
            clock_base = GPIO.LOW
        elif mode == 1:
            # Mode 1 capture of falling edge, propagate on rising clock
            self.write_clock_ve = 0
            self.read_clock_ve  = 1
            # Clock base is low.
            clock_base = GPIO.LOW
        elif mode == 2:
            # Mode 2 capture on rising clock, propagate on falling clock
            self.write_clock_ve = 1
            self.read_clock_ve  = 0
            # Clock base is high.
            clock_base = GPIO.HIGH
        elif mode == 3:
            # Mode 3 capture on falling edge, propagage on rising clock
            self.write_clock_ve = 0
            self.read_clock_ve  = 1
            # Clock base is high.
            clock_base = GPIO.HIGH
        # Set clock and DO as output, DI as input.  Also start clock at its base value.
        self._ft232h.setup_pins({0: GPIO.OUT, 1: GPIO.OUT, 2: GPIO.IN}, {0: clock_base})

    def set_bit_order(self, order):
        """Set order of bits to be read/written over serial lines.  Should be
        either MSBFIRST for most-significant first, or LSBFIRST for
        least-signifcant first.
        """
        if order == MSBFIRST:
            self.lsbfirst = 0
        elif order == LSBFIRST:
            self.lsbfirst = 1
        else:
            raise ValueError('Order must be MSBFIRST or LSBFIRST.')

    def write(self, data):
        """Half-duplex SPI write.  The specified array of bytes will be clocked
        out the MOSI line.
        """
        # Build command to write SPI data.
        command = 0x10 | (self.lsbfirst << 3) | self.write_clock_ve
        logger.debug('SPI write with command {0:2X}.'.format(command))
        # Compute length low and high bytes.
        # NOTE: Must actually send length minus one because the MPSSE engine
        # considers 0 a length of 1 and FFFF a length of 65536
        length = len(data)-1
        len_low  = length & 0xFF
        len_high = (length >> 8) & 0xFF
        self._assert_cs()
        # Send command and length.
        self._ft232h._write(str(bytearray((command, len_low, len_high))))
        # Send data.
        self._ft232h._write(str(bytearray(data)))
        self._deassert_cs()

    def read(self, length):
        """Half-duplex SPI read.  The specified length of bytes will be clocked
        in the MISO line and returned as a bytearray object.
        """
        # Build command to read SPI data.
        command = 0x20 | (self.lsbfirst << 3) | (self.read_clock_ve << 2)
        logger.debug('SPI read with command {0:2X}.'.format(command))
        # Compute length low and high bytes.
        # NOTE: Must actually send length minus one because the MPSSE engine
        # considers 0 a length of 1 and FFFF a length of 65536
        len_low  = (length-1) & 0xFF
        len_high = ((length-1) >> 8) & 0xFF
        self._assert_cs()
        # Send command and length.
        self._ft232h._write(str(bytearray((command, len_low, len_high, 0x87))))
        self._deassert_cs()
        # Read response bytes.
        return bytearray(self._ft232h._poll_read(length))

    def transfer(self, data):
        """Full-duplex SPI read and write.  The specified array of bytes will be
        clocked out the MOSI line, while simultaneously bytes will be read from
        the MISO line.  Read bytes will be returned as a bytearray object.
        """
        # Build command to read and write SPI data.
        command = 0x30 | (self.lsbfirst << 3) | (self.read_clock_ve << 2) | self.write_clock_ve
        logger.debug('SPI transfer with command {0:2X}.'.format(command))
        # Compute length low and high bytes.
        # NOTE: Must actually send length minus one because the MPSSE engine
        # considers 0 a length of 1 and FFFF a length of 65536
        length = len(data)
        len_low  = (length-1) & 0xFF
        len_high = ((length-1) >> 8) & 0xFF
        # Send command and length.
        self._assert_cs()
        self._ft232h._write(str(bytearray((command, len_low, len_high))))
        self._ft232h._write(str(bytearray(data)))
        self._ft232h._write('\x87')
        self._deassert_cs()
        # Read response bytes.
        return bytearray(self._ft232h._poll_read(length))


class I2CDevice(object):
    """Class for communicating with an I2C device using the smbus library.
    Allows reading and writing 8-bit, 16-bit, and byte array values to registers
    on the device."""
    # Note that most of the functions in this code are adapted from this app note:
    #  http://www.ftdichip.com/Support/Documents/AppNotes/AN_255_USB%20to%20I2C%20Example%20using%20the%20FT232H%20and%20FT201X%20devices.pdf
    def __init__(self, ft232h, address, clock_hz=100000):
        """Create an instance of the I2C device at the specified address on the
        specified I2C bus number."""
        self._address = address
        self._ft232h = ft232h
        # Enable clock with three phases for I2C.
        self._ft232h.mpsse_set_clock(clock_hz, three_phase=True)
        # Enable drive-zero mode to drive outputs low on 0 and tri-state on 1.
        # This matches the protocol for I2C communication so multiple devices can
        # share the I2C bus.
        self._ft232h._write('\x9E\x07\x00')
        self._idle()

    def _idle(self):
        """Put I2C lines into idle state."""
        # Put the I2C lines into an idle state with SCL and SDA high.
        self._ft232h.setup_pins({0: GPIO.OUT, 1: GPIO.OUT, 2: GPIO.IN},
                                {0: GPIO.HIGH, 1: GPIO.HIGH})

    def _transaction_start(self):
        """Start I2C transaction."""
        # Clear command buffer and expected response bytes.
        self._command = []
        self._expected = 0

    def _transaction_end(self):
        """End I2C transaction and get response bytes, including ACKs."""
        # Ask to return response bytes immediately.
        self._command.append('\x87')
        # Send the entire command to the MPSSE.
        self._ft232h._write(''.join(self._command))
        # Read response bytes and return them.
        return bytearray(self._ft232h._poll_read(self._expected))

    def _i2c_start(self):
        """Send I2C start signal. Must be called within a transaction start/end.
        """
        # Set SCL high and SDA low, repeat 4 times to stay in this state for a
        # short period of time.
        self._ft232h.output_pins({0: GPIO.HIGH, 1: GPIO.LOW}, write=False)
        self._command.append(self._ft232h.mpsse_gpio() * _REPEAT_DELAY)
        # Now drop SCL to low (again repeat 4 times for short delay).
        self._ft232h.output_pins({0: GPIO.LOW, 1: GPIO.LOW}, write=False)
        self._command.append(self._ft232h.mpsse_gpio() * _REPEAT_DELAY)

    def _i2c_idle(self):
        """Set I2C signals to idle state with SCL and SDA at a high value. Must
        be called within a transaction start/end.
        """
        self._ft232h.output_pins({0: GPIO.HIGH, 1: GPIO.HIGH}, write=False)
        self._command.append(self._ft232h.mpsse_gpio() * _REPEAT_DELAY)

    def _i2c_stop(self):
        """Send I2C stop signal. Must be called within a transaction start/end.
        """
        # Set SCL low and SDA low for a short period.
        self._ft232h.output_pins({0: GPIO.LOW, 1: GPIO.LOW}, write=False)
        self._command.append(self._ft232h.mpsse_gpio() * _REPEAT_DELAY)
        # Set SCL high and SDA low for a short period.
        self._ft232h.output_pins({0: GPIO.HIGH, 1: GPIO.LOW}, write=False)
        self._command.append(self._ft232h.mpsse_gpio() * _REPEAT_DELAY)
        # Finally set SCL high and SDA high for a short period.
        self._ft232h.output_pins({0: GPIO.HIGH, 1: GPIO.HIGH}, write=False)
        self._command.append(self._ft232h.mpsse_gpio() * _REPEAT_DELAY)

    def _i2c_read_bytes(self, length=1):
        """Read the specified number of bytes from the I2C bus.  Length is the
        number of bytes to read (must be 1 or more).
        """
        for i in range(length-1):
            # Read a byte and send ACK.
            self._command.append('\x20\x00\x00\x13\x00\x00')
            # Make sure pins are back in idle state with clock low and data high.
            self._ft232h.output_pins({0: GPIO.LOW, 1: GPIO.HIGH}, write=False)
            self._command.append(self._ft232h.mpsse_gpio())
        # Read last byte and send NAK.
        self._command.append('\x20\x00\x00\x13\x00\xFF')
        # Make sure pins are back in idle state with clock low and data high.
        self._ft232h.output_pins({0: GPIO.LOW, 1: GPIO.HIGH}, write=False)
        self._command.append(self._ft232h.mpsse_gpio())
        # Increase expected number of bytes.
        self._expected += length

    def _i2c_write_bytes(self, data):
        """Write the specified number of bytes to the chip."""
        for byte in data:
            # Write byte.
            self._command.append(str(bytearray((0x11, 0x00, 0x00, byte))))
            # Make sure pins are back in idle state with clock low and data high.
            self._ft232h.output_pins({0: GPIO.LOW, 1: GPIO.HIGH}, write=False)
            self._command.append(self._ft232h.mpsse_gpio() * _REPEAT_DELAY)
            # Read bit for ACK/NAK.
            self._command.append('\x22\x00')
        # Increase expected response bytes.
        self._expected += len(data)

    def _address_byte(self, read=True):
        """Return the address byte with the specified R/W bit set.  If read is
        True the R/W bit will be 1, otherwise the R/W bit will be 0.
        """
        if read:
            return (self._address << 1) | 0x01
        else:
            return self._address << 1

    def _verify_acks(self, response):
        """Check all the specified bytes have the ACK bit set.  Throws a
        RuntimeError exception if not all the ACKs are set.
        """
        for byte in response:
            if byte & 0x01 != 0x00:
                raise RuntimeError('Failed to find expected I2C ACK!')

    def ping(self):
        """Attempt to detect if a device at this address is present on the I2C
        bus.  Will send out the device's address for writing and verify an ACK
        is received.  Returns true if the ACK is received, and false if not.
        """
        self._idle()
        self._transaction_start()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(False)])
        self._i2c_stop()
        response = self._transaction_end()
        if len(response) != 1:
            raise RuntimeError('Expected 1 response byte but received {0} byte(s).'.format(len(response)))
        return ((response[0] & 0x01) == 0x00)

    def writeRaw8(self, value):
        """Write an 8-bit value on the bus (without register)."""
        value = value & 0xFF
        self._idle()
        self._transaction_start()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(False), value])
        self._i2c_stop()
        response = self._transaction_end()
        self._verify_acks(response)

    def write8(self, register, value):
        """Write an 8-bit value to the specified register."""
        value = value & 0xFF
        self._idle()
        self._transaction_start()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(False), register, value])
        self._i2c_stop()
        response = self._transaction_end()
        self._verify_acks(response)

    def write16(self, register, value, little_endian=True):
        """Write a 16-bit value to the specified register."""
        value = value & 0xFFFF
        value_low  = value & 0xFF
        value_high = (value >> 8) & 0xFF
        if not little_endian:
            value_low, value_high = value_high, value_low
        self._idle()
        self._transaction_start()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(False), register, value_low,
                                value_high])
        self._i2c_stop()
        response = self._transaction_end()
        self._verify_acks(response)

    def writeList(self, register, data):
        """Write bytes to the specified register."""
        self._idle()
        self._transaction_start()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(False), register] + data)
        self._i2c_stop()
        response = self._transaction_end()
        self._verify_acks(response)

    def readList(self, register, length):
        """Read a length number of bytes from the specified register.  Results
        will be returned as a bytearray."""
        if length <= 0:
            raise ValueError("Length must be at least 1 byte.")
        self._idle()
        self._transaction_start()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(True), register])
        self._i2c_stop()
        self._i2c_idle()
        self._i2c_start()
        self._i2c_read_bytes(length)
        self._i2c_stop()
        response = self._transaction_end()
        self._verify_acks(response[:-length])
        return response[-length:]

    def readRaw8(self):
        """Read an 8-bit value on the bus (without register)."""
        self._idle()
        self._transaction_start()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(False)])
        self._i2c_stop()
        self._i2c_idle()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(True)])
        self._i2c_read_bytes(1)
        self._i2c_stop()
        response = self._transaction_end()
        self._verify_acks(response[:-1])
        return response[-1]

    def readU8(self, register):
        """Read an unsigned byte from the specified register."""
        self._idle()
        self._transaction_start()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(False), register])
        self._i2c_stop()
        self._i2c_idle()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(True)])
        self._i2c_read_bytes(1)
        self._i2c_stop()
        response = self._transaction_end()
        self._verify_acks(response[:-1])
        return response[-1]

    def readS8(self, register):
        """Read a signed byte from the specified register."""
        result = self.readU8(register)
        if result > 127:
            result -= 256
        return result

    def readU16(self, register, little_endian=True):
        """Read an unsigned 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first)."""
        self._idle()
        self._transaction_start()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(False), register])
        self._i2c_stop()
        self._i2c_idle()
        self._i2c_start()
        self._i2c_write_bytes([self._address_byte(True)])
        self._i2c_read_bytes(2)
        self._i2c_stop()
        response = self._transaction_end()
        self._verify_acks(response[:-2])
        if little_endian:
            return (response[-1] << 8) | response[-2]
        else:
            return (response[-2] << 8) | response[-1]

    def readS16(self, register, little_endian=True):
        """Read a signed 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first)."""
        result = self.readU16(register, little_endian)
        if result > 32767:
            result -= 65536
        return result

    def readU16LE(self, register):
        """Read an unsigned 16-bit value from the specified register, in little
        endian byte order."""
        return self.readU16(register, little_endian=True)

    def readU16BE(self, register):
        """Read an unsigned 16-bit value from the specified register, in big
        endian byte order."""
        return self.readU16(register, little_endian=False)

    def readS16LE(self, register):
        """Read a signed 16-bit value from the specified register, in little
        endian byte order."""
        return self.readS16(register, little_endian=True)

    def readS16BE(self, register):
        """Read a signed 16-bit value from the specified register, in big
        endian byte order."""
        return self.readS16(register, little_endian=False)

class OneWireDevice(object):

    def __init__(self, ft232h, pin, overdrive=False, default_hz=100000, three_phase=True):
        self._pin = pin
        self._ft232h = ft232h
        self._buffer = False
        self._output = None
        self._three_phase = three_phase
        self._default_hz = default_hz

        # Setup the clock to be I2C friendly (100000hz and 3phase by default)
        # We return clock to this after each 1wire command pipeline completes
        self._ft232h.mpsse_set_clock(default_hz, False, three_phase)

        # Set the delays/clock speeds for standard/overdrive timings
        self.enable_overdrive(overdrive)

        # Two ways to delay. dump a byte to tms, or pulse the clock for n
        # bits. A 1 bit pulse seems to take the same as time as a 8bit 
        # dump to TMS? Default is to pulse the clock.
        self._tms_dump    = '\x4a\x01\xff'  # Dump 8 bits to TMS
        self._pb          = '\x8e\x01'      # Pulse clock (1 bits) 
        self._delay       = self._pb

        # MPSSE Command to read GPIO
        self._read_gpio   = '\x81\x83'

        # Set up our GPIO mask for 1-Wire, and leave it high
        self.set_pin(pin, GPIO.OUT, GPIO.HIGH)

    # Return the MPSSE command required to set the clock to a given frequency
    # for the provided delay
    def _get_delay_cmd(self, seconds):
        if seconds == 0:
            clock_hz = 30000000.0
        else:
            clock_hz = ( 1.00 / seconds ) * 2 
        return self._ft232h.mpsse_clock(clock_hz, self._three_phase)

    # Calculate the delay clocks for 1Wire timings
    def enable_overdrive(self, overdrive):
        if overdrive:
            logger.debug('1Wire: Overdrive is enabled')
            # overdrive speeds
            self._overdrive = True
            self._clock_A = self._get_delay_cmd(0.0000010)
            self._clock_B = self._get_delay_cmd(0.0000075)
            self._clock_C = self._get_delay_cmd(0.0000075)
            self._clock_D = self._get_delay_cmd(0.0000025)
            self._clock_E = self._get_delay_cmd(0.0000010)
            self._clock_F = self._get_delay_cmd(0.0000070)
            self._clock_G = self._get_delay_cmd(0.0000025)
            self._clock_H = self._get_delay_cmd(0.0000700)
            self._clock_I = self._get_delay_cmd(0.0000085)
            self._clock_J = self._get_delay_cmd(0.0000400)
        else:
            # standard clock speeds
            logger.debug('1Wire: Overdrive is disabled')
            self._overdrive = False
            self._clock_A = self._get_delay_cmd(0.000006)
            self._clock_B = self._get_delay_cmd(0.000064)
            self._clock_C = self._get_delay_cmd(0.000060)
            self._clock_D = self._get_delay_cmd(0.000010)
            self._clock_E = self._get_delay_cmd(0.000009)
            self._clock_F = self._get_delay_cmd(0.000055)
            self._clock_G = self._get_delay_cmd(0.000000)
            self._clock_H = self._get_delay_cmd(0.000480)
            self._clock_I = self._get_delay_cmd(0.000070)
            self._clock_J = self._get_delay_cmd(0.000410)

        # Default clock speed
        self._clock_default = self._ft232h.mpsse_clock(self._default_hz, self._three_phase)

    # Buffer write commands and then send them to the MPSSE with a flush
    def enable_command_buffer(self):
        if self._buffer:
            raise Exception("Buffering was already enabled!!")
        self._buffer = True
        self._output = None

    # flush buffered commands to the MPSSE
    def flush_command_buffer(self):
        self._buffer = False
        if self._output is not None:
            self._write(self._output)
            self._output = None

    # Write the command pipeline, but append the default_clock before talking to the MPSSE
    def _write(self, string):
        if self._buffer:
            if self._output is None:
                self._output = string
            else:
                self._output += string
            return
        string += str(self._clock_default)
        self._ft232h._write(string)

    def _read(self, length, timeout=5):
        return self._ft232h._poll_read(length, timeout)

    # Set the GPIO to the state requested and update the self.low, self.high
    # values to take into account the changed pin.
    def set_pin(self, pin, mode, value):

        # Update pin state to that requested
        if pin is not self._pin:
            self._ft232h.setup(pin, mode)
            self._ft232h.output(pin, value)

        # Calculate the mask for the GPIO when 1-wire is low
        self._ft232h.setup_pins({self._pin: GPIO.OUT},{self._pin: GPIO.LOW}, write=False)
        self._low = self._ft232h.mpsse_gpio()

        # Calculate the mask for the GPIO when 1-wire is high
        self._ft232h.setup_pins({self._pin: GPIO.IN},{self._pin: GPIO.HIGH}, write=False)
        self._high = self._ft232h.mpsse_gpio()

    # Here begins the 1-wire stuff

    # Send a 1-wire reset on the GPIO, This makes all slaves listen up for commands.
    # It also detects the presance of the slaves. If nothing responds, then no devices
    # are connected and we return false.
    def reset(self):

        logger.debug("1Wire: Reset")
        commands = self._clock_H + self._low + self._delay + self._high + self._clock_I +\
                    self._delay + self._read_gpio + self._clock_J + self._delay + \
                    self._read_gpio

        self._write(str(commands))
        present = self._read(4)

        if present == '\xff'*4:
            if self._overdrive:
                logger.debug("1Wire: No Devices Present. Disabling Overdrive")
                self.enable_overdrive(False)
                return self.reset()
            else:
                logger.debug("1Wire: No Devices Present")
                return False
        else:
            logger.debug("1Wire: Devices Present")
            return True

    # Write a bit to the 1-wire bus, either a 1 or a 0
    def write_bit(self, bit):
        if bit:
            commands = self._clock_A + self._low + self._delay + self._high + \
            self._clock_B + self._delay
        else:
            commands = self._clock_C + self._low + self._delay + self._high + \
            self._clock_D + self._delay

        self._write(str(commands))

    def read_command(self, bits=1):
        for i in range(bits):
            commands = self._clock_A + self._low + self._delay + self._high + \
            self._clock_E + self._delay + self._read_gpio + self._clock_F + \
            self._delay
            self._write(str(commands))

    def read_response(self, bits=1):
        states = []
        read = self._read(2 * bits)
        for i in range(0,2*bits,2):
            bit = bytearray([read[i], read[i+1]])
            states.append( struct.unpack("H", str(bit))[0] >> self._pin & 01 )
        if bits == 1:
            return states.pop()
        return states

    # Read a bit from the 1-wire bus.
    def read_bit(self):
        self.read_command()
        return self.read_response()

    # Use the write_bit function to write bytes out to the bus
    def write_byte(self, byte):
        manage_buffer = self._buffer == False
        if manage_buffer:
            self.enable_command_buffer()
        for i in range(8):
            self.write_bit(byte & 1)
            byte >>= 1
        if manage_buffer:
            self.flush_command_buffer()

    # write multiple bytes to the bus
    def write_bytes(self, data):
        for byte in data:
            self.write_byte(byte)

    # Use the read_bit function to read bytes from the bus
    def read_byte(self):
        byte = 0
        manage_buffer = self._buffer == False
        if manage_buffer:
            self.enable_command_buffer()
        self.read_command(8)
        if manage_buffer:
            self.flush_command_buffer()
        bits = self.read_response(8)
        for i in range(8):
            byte |= bits[i] << i
        return byte 

    # Read multiple bytes from the 1-wire bus
    def read_bytes(self, count):
        data = bytearray(count)
        for i in range(count):
            data[i] = self.read_byte()
        return data

    # read multiple bits from the 1-wire bus. Used for device discovery
    def read_bits(self, count):
        bits = []
        for i in range(count):
            bits.append( self.read_bit() )
        return bits
    
    # There is only one device on the bus, so ask it to identify itself.
    def rom_read(self):
        rom = bytearray(8)
        self.write_byte(0x33)
        for i in range(8):
            rom[i] = self.read_byte()
        return rom 

    # There is only one device on the bus so skip ROM matching.
    def skip_rom(self):
        self.write_byte(0xcc)

    # Target the ROM specified
    def _match_rom(self, rom):
        if type(rom) is str:
            rom = self.string2bytes(rom)
        self.write_byte(0x55)
        self.write_bytes(rom)

    # Address the ROM if given, else perform a skip_rom()
    def address_rom(self, rom):
        if rom is None:
            self.skip_rom()
        else:
            self._match_rom(rom)

    # Search for ROMs on the 1-wire bus
    def search_roms(self):
        roms_found = []
        partials = [ [] ]
        logger.debug("Search Start")
        while len(partials) > 0:
            rom = partials.pop()
            roms_found.append( self.bytes2string(self._search(rom, partials)) )
        logger.debug("Search Complete")
        return roms_found

    # When replaying the partial rom, flush the search out to the MPSSE every 10 bits
    # improves performance.
    def _search_flush_rom(self, count):
        self.flush_command_buffer()
        self.read_response(2*count)
        return 0

    # Do the search for each partial ROM
    def _search(self, rom=[], partials=[]):

        if self.reset() is False:
            return

        # Dump any partial rom to the MPSSE in 10bit chunks
        self.enable_command_buffer()
        self.write_byte(0xf0)
        count = 0
        for bit in rom:
            if count == 10:
                count = self._search_flush_rom(10)
                self.enable_command_buffer()
            count += 1
            self.read_command(2)
            self.write_bit(bit)
        self._search_flush_rom(count)

        # Continue the search from where we are.
        for i in range(64 - len(rom)):
            bits = self.read_bits(2)
            if bits[0] != bits[1]:
                rom.append(bits[0])
                self.write_bit(bits[0])
            elif bits == [False, False]:
                np = list(rom)
                np.append(True)
                partials.append( np )
                rom.append(False)
                self.write_bit(False)
            else:
                raise Exception("Search Failed. Device Comms Interrupted")

        complete = bytearray(8)
        for i in range(8):
            byte = 0
            for o in range(8):
                bit = rom[(i*8)+o]
                byte |= bit << o
            complete[i] = byte

        logger.debug("Search Found: ROM {}".format( self.bytes2string(complete)))
        if self.crc(complete) is not 0x00:
            raise Exception("CRC Check Failed")
        return complete

   # Return an a string representation of the device ROM
    def bytes2string(self, bytesarray):
        return ":".join("{:02x}".format(c) for c in bytesarray)

    # Convert a hexadecimal string to bytes
    def string2bytes(self, string):
        return bytearray(codecs.decode(string.replace(":",""),"hex"))

    # Calculate CRC, result should be 0x00
    def crc(self, data):
        poly = 0x8c # x8,x5,x4,+ 1 inverse of 0x131 & 0xff
        crc = 0x00
        for byte in data:
            for bit in range(8):
                # When bit is on, shift and xor, else just shift
                if ( byte ^ crc) & 0x01:
                    crc >>= 1
                    crc ^= poly
                else:
                    crc >>= 1
                byte >>= 1
        return crc

