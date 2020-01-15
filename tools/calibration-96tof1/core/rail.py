""" ** ToFRail: TOF rail control python module**

    **Author**: Antonio Rubio Salcedo <antonio.salcedo@analog.com>

    **Code File**: `ToFRail.py`

=================

- **Edits:**
    -   19 June 2018 antonio.salcedo@analog.com
            Initial version with same functionallity than rail library for the GUI tool

=================


- **Generic libraries**

    import serial

=================

"""

import serial
import time

class Rail:
    """
    This class is used to provide control for the tof rail.
    
    :param COMPort: Communication port addres ( Examples: "COM2", "/dev/tty0")
    :type COMPort: str
    """
    def __init__( self, COMPort):
        self.port = COMPort
        self.offset = 0
        self.distance = 0
        self.EndDistance = 0
        self.bBusy = False

        self.ser = serial.Serial( '/dev/ttyUSB0', 9524, timeout=1)


    def moveRail(self, distance):
        """
        Moves the rail to the given distance position in cm.
        :param distance: distance in cm.
        :type distance: int
        """
        return self.moveRailMM( distance*10)

    def moveRailMM(self, distance):
        """
        Moves the rail to the given distance position in mm.
        :param distance: distance in mm.
        :type distance: int
        """
        if(self.checkBusyStatus()):
            #print("moveRailMM() Error : 3")
            return 3

        return self.moveTo( distance)

    def writeRailOffset( self, offset):
        """
        Sets rail offset and performs a ZERO operation moving the rail target until hit the zero position.
        :param offset: offset distance in mm.
        :type offset: int
        """
        if(self.checkBusyStatus()):
            #print("writeRailOffset() Error : 3")
            return 3
        self.Zero()
        self.setoffset(offset)

    def mForward( self, steps):
        """
        Moves away from actual position in mm.
        :param steps: distance to move in mm.
        :type steps: int
        """
        self.ser.write(b"\xae\xaa\xf0")
        self.ser.write(b"\xaa")
        self.ser.write( bytes([steps>>8]))
        self.ser.write( bytes([steps & 0xFF]))
        self.ser.write( b"\xfa")

    def mBack( self, steps):
        """
        Moves toward from actual position in mm.
        :param steps: distance to move in mm.
        :type steps: int
        """
        self.ser.write(b"\xaf\xaa\xf0")
        self.ser.write(b"\xaa")
        self.ser.write( bytes([steps>>8]))
        self.ser.write( bytes([steps & 0xFF]))
        self.ser.write( b"\xfa")

    def Zero(self):     
        """
        Moves toward from actual position in mm.
        :param steps: distance to move in mm.
        :type steps: int
        """

        self.ser.write(b"\xab")
        self.ser.write(b"\xaa")

        temp = self.ReadChar()

        if( temp == b"\xf1" or temp == b"\xf2"):
            return 1
        else:
            self.bBusy = True
            return 0

    def moveTo( self, distance):

        #print( "moveTo(): distance = " + str(distance))

        while( self.checkBusyStatus() == True):
            pass
      

        #print("moveTo(): sending move command to Rail")
        self.distance = distance
        self.ser.write( b"\xbc\xaa\xf0")
        self.ser.write( bytes([distance>>8]))
        self.ser.write( bytes([distance & 0xFF]))
        self.ser.write( b"\xfa")

        self.bBusy = True
        return 1

    def checkBusyStatus(self):

        if( self.bBusy == True):
            temp = self.ReadChar()
            if( temp == b"\xf1" or temp == b"\xf2"):
                self.bBusy = False
        return self.bBusy

    def setoffset(self, offset):
        while( self.checkBusyStatus() == True):
            pass

        self.offset = offset

        self.ser.write( b"\xbd\xaa\xf0")
        self.ser.write( bytes([offset>>8]))
        self.ser.write( bytes([offset & 0xFF]))
        self.ser.write( b"\xfa")

    def current_distance( self):

        self.ser.write(b"\xbe\xaa")

        buffer = self.ReadComm()
        distance = int.from_bytes( buffer, byteorder = 'big', signed = False)
        '''
        ret = self.ser.read()
        if( len(ret) <= 0):
            print("current_distance(): no characters received from UART")

        distance = int.from_bytes( ret, byteorder = 'big', signed = False)
        '''
        #print("current_distance() : distance = " + str(distance))

        return distance


    def set_stepsize( self, stepsize):
        self.ser.write("b\xbf\xaa\xf0")
        self.ser.write( bytes([stepsize>>8]))
        self.ser.write( bytes([stepsize & 0xFF]))
        self.ser.write( b"\xfa")
  

    def ReadComm(self):
        temp = bytes()

        while(True):
            rxchars = self.ser.read()

            if( len(rxchars) > 0):
                temp = temp + rxchars
            else:
                break

        return temp

    def ReadChar(self):
        temp = bytes()

        while(True):
            rxchars = self.ser.read()

            if( len(rxchars) > 0):
                temp = rxchars
            else:
                break

        return temp

    def close(self):

        self.ser.close()

if __name__ == "__main__":

    rail = Rail( "COM2")

    rail.writeRailOffset(100)
    while( rail.moveRail(50) !=1):
        pass
    while( rail.moveRail(100) != 1):
        pass

    rail.close()