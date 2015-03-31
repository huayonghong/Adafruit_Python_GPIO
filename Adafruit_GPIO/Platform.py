# Copyright (c) 2014 Adafruit Industries

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import platform

from collections import OrderedDict

# Platform identification constants.
UNKNOWN = 0
RASPBERRY_PI = 1
BEAGLEBONE_BLACK = 2


class SystemFactory(object):
    """
    Collection of all system configuration properties.
    It is used as a factory to create any component
    """

    def __init__(self, cpu_info = {}, name = "Unknown"):
        """
        Create SystemFactory from data in cpu_info
        :param cpu_info: dictionary with keys and values in lower case
        :param name: a mark for representation only
        :return: factory to create components
        """
        self.hardware = cpu_info.get('hardware', None)
        self.revision = cpu_info.get('revision', None)
        self.model_name = cpu_info.get('model name', None)
        self.name = name
        if self.hardware and self.revision and self.model_name:
            if self.hardware == 'bcm2708':
                # Raspberry Pi 1 (model A, B, A+, B+)
                self.platform_id = RASPBERRY_PI
            elif self.hardware == 'bcm2709':
                # Raspberry Pi 2 (model B+)
                self.platform_id = RASPBERRY_PI
            elif 'am33' in self.hardware and 'v7l' in self.model_name:
                self.platform_id = BEAGLEBONE_BLACK
            else:
                self.platform_id = UNKNOWN
        else:
            self.platform_id = UNKNOWN

    def __str__(self):
        return "<{} - {} - {} - {}>".format(self.hardware, self.revision, self.model_name, self.name)

    def get_platform_pwm(self, **keywords):
        """Attempt to return a PWM instance for the platform which the code is being
        executed on.  Currently supports only the Raspberry Pi using the RPi.GPIO
        library and Beaglebone Black using the Adafruit_BBIO library.  Will throw an
        exception if a PWM instance can't be created for the current platform.  The
        returned PWM object has the same interface as the RPi_PWM_Adapter and
        BBIO_PWM_Adapter classes.
        """
        if self.platform_id == RASPBERRY_PI:
            import RPi.GPIO
            from Adafruit_GPIO.PWM import RPi_PWM_Adapter
            return RPi_PWM_Adapter(RPi.GPIO, **keywords)
        elif self.platform_id == BEAGLEBONE_BLACK:
            import Adafruit_BBIO.PWM
            from Adafruit_GPIO.PWM import BBIO_PWM_Adapter
            return BBIO_PWM_Adapter(Adafruit_BBIO.PWM, **keywords)
        elif self.platform_id == UNKNOWN:
            raise RuntimeError('No PWM implementation found for {}.'.format(self.name))

    def get_platform_gpio(self, **keywords):
        """Attempt to return a GPIO instance for the platform which the code is being
        executed on.  Currently supports only the Raspberry Pi using the RPi.GPIO
        library and Beaglebone Black using the Adafruit_BBIO library.  Will throw an
        exception if a GPIO instance can't be created for the current platform.  The
        returned GPIO object is an instance of BaseGPIO.
        """
        if self.platform_id == RASPBERRY_PI:
            import RPi.GPIO
            from Adafruit_GPIO.GPIO import RPiGPIOAdapter
            return RPiGPIOAdapter(RPi.GPIO, **keywords)
        elif self.platform_id == BEAGLEBONE_BLACK:
            import Adafruit_BBIO.GPIO
            from Adafruit_GPIO.GPIO import AdafruitBBIOAdapter
            return AdafruitBBIOAdapter(Adafruit_BBIO.GPIO, **keywords)
        elif self.platform_id == UNKNOWN:
            raise RuntimeError('No GPIO implementation found for {}.'.format(self.name))

    def get_default_bus(self):
            """Return the default bus number based on the device platform.  For a
            Raspberry Pi either bus 0 or 1 (based on the Pi revision) will be returned.
            For a Beaglebone Black the first user accessible bus, 1, will be returned.
            """
            if self.platform_id == RASPBERRY_PI and self.revision:
                # Revision list available at: http://elinux.org/RPi_HardwareHistory#Board_Revision_History
                # Match a line of the form "Revision : 0002" while ignoring extra
                # info in front of the revision (like 1000 when the Pi was over-volted).
                rev = self.revision[-4:]
                if rev in ['0000', '0002', '0003']:
                    # Revision 1 Pi uses I2C bus 0.
                    return 0
                else:
                    # Revision 2 Pi uses I2C bus 1.
                    return 1
            elif self.platform_id == BEAGLEBONE_BLACK:
                # Beaglebone Black has multiple I2C buses, default to 1 (P9_19 and P9_20).
                return 1
            else:
                raise RuntimeError('Could not determine default I2C bus for platform for {}.'.format(self.name))

    def get_i2c_device(self, address, busnum=None, **kwargs):
        """Return an I2C device for the specified address and on the specified bus.
        If busnum isn't specified, the default I2C bus for the platform will attempt
        to be detected.
        """
        from Adafruit_GPIO.I2C import Device
        if busnum is None:
            busnum = self.get_default_bus()
        return Device(address, busnum, **kwargs)

    def require_repeated_start(self):
        """Enable repeated start conditions for I2C register reads.  This is the
        normal behavior for I2C, however on some platforms like the Raspberry Pi
        there are bugs which disable repeated starts unless explicitly enabled with
        this function.  See this thread for more details:
          http://www.raspberrypi.org/forums/viewtopic.php?f=44&t=15840
        """
        if self.platform_id == RASPBERRY_PI:
            import subprocess
            # On the Raspberry Pi there is a bug where register reads don't send a
            # repeated start condition like the kernel smbus I2C driver functions
            # define.  As a workaround this bit in the BCM2708 driver sysfs tree can
            # be changed to enable I2C repeated starts.
            subprocess.check_call('chmod 666 /sys/module/i2c_bcm2708/parameters/combined', shell=True)
            subprocess.check_call('echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined', shell=True)
        # Other platforms are a no-op because they (presumably) have the correct
        # behavior and send repeated starts.


def read_proc_info():
    """
    Read /proc/cpuinfo, make lower case and parse it into a dictionary.
    In case of many CPUs, the last value for same key is used
    :return: parsed values or empty dictionary
    """
    import os.path
    fname = "/proc/cpuinfo"
    if os.path.isfile(fname):
        with open(fname, 'r') as infile:
            raw = infile.read().lower()
            return OrderedDict([map(str.strip, line.split(':')) for line in raw.splitlines() if line.strip()])
    else:
        # Mac or Windows might be used for development
        return OrderedDict()


def create_system_config():
    """
    Read 'proc/cpuinfo' and configure SystemFactory
    :return: factory to create components
    """
    cpu_info_dict = read_proc_info()
    return SystemFactory(cpu_info=cpu_info_dict, name=platform.platform())
