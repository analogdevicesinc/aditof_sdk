#
# BSD 3-Clause License
#
# Copyright (c) 2019, Analog Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
""" ** ToFRail: TOF rail control python module**

    **Author**: Harshvardhan Bhatia

=================

- **Edits:**
    -   19 June 2018 antonio.salcedo@analog.com
            Inital 

=================


- **Generic libraries**

    import serial
    import modbus_tk

=================

"""

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import numpy as np
import struct


class Rail:
    """
    This class is used to provide control for the tof rail.
    
    :param COMPort: Communication port addres ( Examples: "COM2", "/dev/tty0")
    :type COMPort: str
    """
    def __init__(self, COMPort, verbose=False):
        self.port = COMPort
        self.offset = 0

        self.logger = modbus_tk.utils.create_logger("console")

        self.master = modbus_rtu.RtuMaster(serial.Serial('/dev/ttyUSB0', baudrate=115200, bytesize=8, parity='E', stopbits=1, xonxoff=0))
        self.master.set_timeout(5.0)
        self.master.set_verbose(verbose)
        self.logger.info("connected")

    def moveRail(self, distance):
        """
        Moves the rail to the given distance position in cm.
        :param distance: distance in cm.
        :type distance: int
        """
        return self.moveRailMM(distance*10)

    def moveRailMM(self, distance):
        """
        Moves the rail to the given distance position in mm.
        :param distance: distance in mm.
        :type distance: int
        """
        info = 'Moving rail to: ' + str(distance)
        self.logger.info(info)
        
        steps = (distance - self.offset) * 20
        self.write_absolute_position(1, steps)
        self.start_operation(1)
        while self.check_move(1):
            x = 1
        return self.actual_position_rail()

    def writeRailOffset(self, offset):
        """
        Sets rail offset and performs a ZERO operation moving the rail target until hit the zero position.
        :param offset: offset distance in mm.
        :type offset: int
        """
        self.offset = offset

    def Zero(self):     
        """
        Moves toward from actual position in mm.
        :param steps: distance to move in mm.
        :type steps: int
        """
        self.moveRailMM(0)
        
    def rotate(self, deg):
        """
        Rotates breadboard on rail by deg
        :param deg: degrees .01 precision
        :type steps: float
        """
        info = 'Rotating to: ' + str(deg)
        steps = 100 * deg
        steps = int(steps)
        self.logger.info(info)
        self.write_absolute_position(2, steps)
        self.start_operation(2)
        while self.check_move(2):
            x = 1
        return self.actual_position_bb()
        
    def check_alarm_code(self, slave_addr):
        """
        Checks driver present alarm code and returns 1 if error found. 
        :param slave_addr: slave address
        :type slave_addr: int
        :
        """
        data = self.master.execute(slave_addr, cst.READ_HOLDING_REGISTERS, 0x0080, 2)
        self.logger.info(data)
        return
        
    def clear_alarm_code(self, slave_addr):
        """
        Clears driver alarm code to allow operation 
        :param slave_addr: slave address
        :type slave_addr: int
        :
        """
        self.logger.info(self.master.execute(slave_addr, cst.READ_HOLDING_REGISTERS, 0x0180, output_value=[upper, lower]))
    
    def actual_position_rail(self):
        steps = self.read_register(1, 0x00CC, 2)
        mm = steps[0]/20
        return mm
        
    def actual_position_bb(self):
        steps = self.read_register(2, 0x00CC, 2)
        deg = steps[0]/100
        return deg
    
    def reg_to_int(self, upper, lower):
        ret = upper << 16
        ret = ret + lower
        return ret
    
    def read_register(self, slave_addr, reg, num_reg):
        if (num_reg < 2) | (num_reg%2 == 1):
            self.logger.error("All registers are 32 bit - Minumum 2 registers must be read, even count only")
        ret = []
        reg_list = self.master.execute(slave_addr, cst.READ_HOLDING_REGISTERS, reg, num_reg)
        for i in np.arange(0, len(reg_list), 2):
            ret.append(self.reg_to_int(reg_list[i], reg_list[i+1]))
            i += 1
        for i, x in enumerate(ret):
            ret[i] = struct.unpack('l', struct.pack('L', x & 0xffffffff))[0]
        return ret
            
    def write_absolute_position(self, slave_addr, steps):
        upper = steps >> 16
        lower = steps & 0xFFFF
        self.logger.info(self.master.execute(slave_addr, cst.WRITE_MULTIPLE_REGISTERS, 0x1802, output_value=[upper, lower]))

    def start_operation(self, slave_addr):
        self.logger.info(self.master.execute(slave_addr, cst.WRITE_MULTIPLE_REGISTERS, 0x007C, output_value=[0,8]))
        self.logger.info(self.master.execute(slave_addr, cst.WRITE_MULTIPLE_REGISTERS, 0x007C, output_value=[0,0]))
        
        
    def check_move(self, slave_addr):
        reg = 126
        data = self.read_register(slave_addr, reg, 2)
        if (data[0] & 0x00002000):
            return True
        return False
        
    def close(self):
        self.master.close()

if __name__ == "__main__":

    rail = Rail("ttyUSB0")

    rail.writeRailOffset(150)
    rail.moveRailMM(0)

    rail.close()