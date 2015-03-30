# Copyright (c) 2014 Adafruit Industries
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
import unittest
import sys
from mock import patch

import Adafruit_GPIO.Platform as Platform
from Adafruit_GPIO.Platform import SystemFactory


class MockFactory(SystemFactory):
    def __init__(self, chip_id):
        self.platform_id = chip_id
        self.name = "Mock-config"


class TestCreateSystemConfig(unittest.TestCase):

    def test_read_proc_info(self):
        p = Platform.read_proc_info()
        if sys.platform == 'linux':
            self.assertTrue(len(p) > 10)
        else:
            self.assertTrue(len(p) == 0)

    def test_read_proc_info_beaglebone(self):
        if sys.platform == 'linux':
            with patch('__builtin__.open') as mock_open:
                handle = mock_open.return_value.__enter__.return_value
                content = """
                    processor	: 0
                    model name	: ARMv7 Processor rev 2 (v7l)
                    BogoMIPS	: 298.24
                    Features	: swp half thumb fastmult vfp edsp thumbee neon vfpv3 tls
                    CPU implementer	: 0x41
                    CPU architecture: 7
                    CPU variant	: 0x3
                    CPU part	: 0xc08
                    CPU revision	: 2

                    Hardware	: Generic AM33XX (Flattened Device Tree)
                    Revision	: 0000
                    Serial		: 0000000000000000
                """
                handle.read.return_value = content
                p = Platform.read_proc_info()
                self.assertEqual(len(p), 12)
                self.assertEqual(p['model name'], 'ARMv7 Processor rev 2 (v7l)'.lower(), 'The values must be lower case')
                self.assertEqual(p['hardware'], 'Generic AM33XX (Flattened Device Tree)'.lower(),
                                 'The keys must be lower case')

    def test_read_proc_info_intel(self):
        if sys.platform == 'linux':
            with patch('__builtin__.open') as mock_open:
                handle = mock_open.return_value.__enter__.return_value
                content = """
                    processor	: 0
                    vendor_id	: GenuineIntel
                    cpu family	: 6
                    model		: 42
                    model name	: Intel(R) Core(TM) i3-2310M CPU @ 2.10GHz
                    cpu MHz		: 800.000
                    cache size	: 3072 KB
                    power management:

                    processor	: 1
                    vendor_id	: GenuineIntel
                    cpu family	: 6
                    model		: 42
                    model name	: Intel(R) Core(TM) i3-2310M CPU @ 2.10GHz
                    cpu MHz		: 800.000
                    cache size	: 3072 KB
                    power management:

                """
                handle.read.return_value = content
                p = Platform.read_proc_info()
                self.assertEqual(len(p), 8)
                self.assertEqual(p['power management'], '')

    def test_create_system_config(self):
        config = Platform.create_system_config()
        self.assertTrue(str(config).startswith('<'))

    def test_create_system_config_without_data(self):
        config = SystemFactory()
        self.assertEqual(config.platform_id, Platform.UNKNOWN)

    def test_create_raspberry_1(self):
        cpu_dict = {'hardware': 'bcm2708', 'revision': '0002', 'model name': 'arm'}
        config = SystemFactory(cpu_dict)
        self.assertEqual(config.platform_id, Platform.RASPBERRY_PI)

    def test_create_raspberry_2(self):
        cpu_dict = {'hardware': 'bcm2709', 'revision': '0002', 'model name': 'arm'}
        config = SystemFactory(cpu_dict)
        self.assertEqual(config.platform_id, Platform.RASPBERRY_PI)

    def test_create_beaglebone(self):
        cpu_dict = {'hardware': 'Generic AM33XX (Flattened Device Tree)'.lower(),
                    'revision': '0000', 'model name': 'ARMv7 Processor rev 2 (v7l)'.lower()
                    }
        config = SystemFactory(cpu_dict)
        self.assertEqual(config.platform_id, Platform.BEAGLEBONE_BLACK)

    def test_create_unexpected(self):
        cpu_dict = {'hardware': 'foo', 'revision': '0001', 'model name': 'bar'}
        config = SystemFactory(cpu_dict)
        self.assertEqual(config.platform_id, Platform.UNKNOWN)

    def test_get_platform_pwm_unknown(self):
        try:
            MockFactory(Platform.UNKNOWN).get_platform_pwm()
            self.fail('Unknown platform must be created.')
        except Exception as inst:
            self.assertEqual(inst.message, 'No PWM implementation found for Mock-config.')

    def test_get_platform_gpio_unknown(self):
        try:
            MockFactory(Platform.UNKNOWN).get_platform_gpio()
            self.fail('Unknown platform must be created.')
        except Exception as inst:
            self.assertEqual(inst.message, 'No GPIO implementation found for Mock-config.')

