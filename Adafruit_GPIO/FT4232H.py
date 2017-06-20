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

import ftdi1 as ftdi

import Adafruit_GPIO.GPIO as GPIO
import Adafruit_GPIO.FT232H as FT232H

#import GPIO
#import Adafruit_GPIO.FT232H

logger = logging.getLogger(__name__)

FT4232H_VID = 0x0403
FT4232H_PID = 0x6011

def use_FT4232H():
	FT232H.use_FT232H();


def enumerate_device_serials(vid=FT4232H_VID, pid=FT4232H_PID):
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
            if serial is not None:
                devices.append(serial)
            device_list = device_list.next
        return devices
    finally:
        # Make sure to clean up list and context when done.
        if device_list is not None:
            ftdi.list_free(device_list)
        if ctx is not None:
            ftdi.free(ctx)


class FT4232H(FT232H.FT232H):

    def __init__(self, vid=FT4232H_VID, pid=FT4232H_PID, serial=None, iface=ftdi.INTERFACE_B, useMPSSE=True):
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

        ftdi.set_interface(self._ctx, iface);

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

        ftdi.set_interface(self._ctx, iface);

        self._mpsse_enable()
        self._mpsse_sync();

        # Initialize all GPIO as inputs.
        self._write('\x80\x00\x00\x82\x00\x00')
        self._direction = 0x0000
        self._level = 0x0000