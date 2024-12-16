import serial
import serial.tools.list_ports

import time

from enum import Enum

RO = 0x01   # Read Output (signal) Y
RI = 0x02   # Read Input (signal) X
RD = 0x03   # Read Data D - cannot read X and Y
WO = 0x05   # Write Output Y - forced signal output
EB = 0x08   # Echo Back
WA = 0x0F   # Write all Y contacts
WD = 0x10   # Write Data D

Y1F = Y17 = X4F = X47 = 1 << 7  # bit 7
Y1E = Y16 = X4E = X46 = 1 << 6  # bit 6
Y1D = Y15 = X4D = X45 = 1 << 5  # bit 5
Y1C = Y14 = X4C = X44 = 1 << 4  # bit 4
Y1B = Y13 = X4B = X43 = 1 << 3  # bit 3
Y1A = Y12 = X4A = X42 = 1 << 2  # bit 2
Y19 = Y11 = X49 = X41 = 1 << 1  # bit 1
Y18 = Y10 = X48 = X40 = 1 << 0  # bit 0

DEBUG_LECP6 = False

class LECP6Response(Enum):
    VALID = 0
    ERROR = 1
    CRC = 2

class LECP6CMD(Enum):
    ENABLE = "ENABLE"
    HOME = "HOME"
    HOME_FINISHED = "HOME FINISHED"
    START = "START"
    READ_INPUTS = "READ INPUTS"
    READ_OUTPUTS = "READ OUTPUTS"
    SEND_DATA = "SEND DATA"

def calculate_crc16(data):
    """
    Calculate the CRC-16 checksum for a given data array.

    This function computes the Cyclic Redundancy Check (CRC-16) value using
    the Modbus polynomial 0xA001. It ensures data integrity during communication
    by detecting errors in transmitted data.

    Parameters:
        data (iterable of int): A sequence of bytes (values between 0 and 255) for which
                                the CRC-16 checksum is calculated.

    Returns:
        int: The computed CRC-16 checksum as a 16-bit integer.

    Example:
        >>> data = [0x01, 0x03, 0x00, 0x00, 0x00, 0x0A]
        >>> calculate_crc16(data)
        50651  # Equivalent to 0xC5DB
    """
         
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


class LECP6Serial:
    def __init__(self, port : str, baudrate : int = 38400, timeout : float = 0.1, CTRL_ID : int = 0x01):
        """
        A class to manage serial communication with the LECP6 SMC actuator controller.

        This class provides methods for sending commands, moving the actuator to a target
        position, and monitoring status signals via serial communication.

        Parameters:
            port (str): The name of the serial port (e.g., "COM4").
            baudrate (int): The baud rate for serial communication (default: 38400).
            timeout (float): Timeout in seconds for serial operations (default: 0.1).
            CTRL_ID (int): Controller ID used in communication (default: 0x01).
        """
        self.CTRL_ID = CTRL_ID

        #commands
        self.commands = {}
        
        self.commands["ENABLE"] =                   [CTRL_ID, WO, 0x00, 0x30, 0xFF, 0x00]
        self.commands["RESET"] =                    [CTRL_ID, WO, 0x00, 0x1B, 0xFF, 0x00]
        self.commands["SERVO ON"] =                 [CTRL_ID, WO, 0x00, 0x19, 0xFF, 0x00]
        self.commands["SERVO OFF"] =                [CTRL_ID, WO, 0x00, 0x19, 0x00, 0x00]
        self.commands["HOME"] =                     [CTRL_ID, WO, 0x00, 0x1C, 0xFF, 0x00]
        self.commands["HOME FINISHED"] =            [CTRL_ID, WO, 0x00, 0x1C, 0x00, 0x00]
        self.commands["START"] =                    [CTRL_ID, WD, 0x91, 0x00, 0x00, 0x01, 0x02, 0x10, 0x00]

        self.commands["READ INPUTS"] =              [CTRL_ID, RI, 0x00, 0x40, 0x00, 0x10]
        # status flags: X40 -> X4F
        # BYTE 1 RESPONSE:
        # X47   X46   X45   X44   X43   X42   X41   X40
        # ---   ---   OUT5  OUT4  OUT3  OUT2  OUT1  OUT0

        # BYTE 2 RESPONSE:
        # X4F   X4E   X4D   X4C   X4B   X4A   X49   X48
        # ALARM ESTOP WAREA AREA  INP   SETON SVRE  BUSY
        
        self.commands["READ OUTPUTS"] =              [CTRL_ID, RO, 0x00, 0x10, 0x00, 0x10]
        # state change flags: Y10 -> Y1F
        # BYTE 1 RESPONSE:
        # Y17   Y16   Y15   Y14   Y13   Y12   Y11   Y10
        # ---   ---   IN5   IN4   IN3   IN2   IN1   IN0

        # BYTE 2 RESPONSE:
        # Y1F   Y1E   Y1D   Y1C   Y1B   Y1A   Y19   Y18
        # ---   JOG+  JOG-  SETUP RESET DRIVE SVON  HOLD


        self.commands["SEND DATA"] =                [CTRL_ID, WD, 0x91, 0x02, 0x00, 0x10, 0x20]

        self.commands_op = {}

        for key in self.commands.keys():
            self.commands_op[key] = self.commands[key][1]

            if key != "SEND DATA":
                crc = calculate_crc16(self.commands[key])
                crc_low = crc & 0xFF
                crc_high = (crc >> 8) & 0xFF
                self.commands[key].append(crc_low)
                self.commands[key].append(crc_high)
        
        self.ser = None

        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        except serial.SerialException as e:
            print(f"Failed to open serial port {port}: {e}")
            return
        

        # example for sending direct instruction of position and speed: 
        #   - ENABLE,                   SETS Y30 = 1
        #   - SERVO ON, CHECK SVRE      SETS Y19 = 1, WAITS X49 == 1
        #   - HOME, CHECK SETON,        SETS Y1C = 1, WAITS X4A == 1
        #   - HOME FINISHED,            SETS Y1C = 0
        #   - SEND DATA,                WRITE D9102 TO D9111
        #   - START,                    WRITE 0x0100 TO D9100

        init_cmds = ["ENABLE", "SERVO ON"]

        for init_cmd in init_cmds:
            response = self.send_cmd(init_cmd)
            if DEBUG_LECP6:
                print(self.process_response(init_cmd, response))
    
        
        self.wait_till_X_set(1, X49)

        self.send_cmd("HOME")
        
        # self.wait_till_X_set(1, X4A)
        self.wait_till_X_set(1, X4B)

        self.send_cmd("HOME FINISHED")


    def send_cmd(self, cmd_name, data = None):
        """
        Send a command to the controller over the serial interface.

        Parameters:
            cmd_name (str or LECP6CMD): The name of the command to send.
            data (list of int, optional): Additional data to send with the "SEND DATA" command.

        Returns:
            str: The response received from the controller as a hexadecimal string.
        """

        if isinstance(cmd_name, LECP6CMD):
            cmd_name = cmd_name.value

        if cmd_name == "SEND DATA":
            if data is None:
                print("No data to send")
                return "0"
                        
            temp_cmd = self.commands[cmd_name] + data

            crc = calculate_crc16(temp_cmd)
            crc_low = crc & 0xFF
            crc_high = (crc >> 8) & 0xFF

            temp_cmd.append(crc_low)
            temp_cmd.append(crc_high)

            cmd_bytes = bytes(temp_cmd)
        else:
            cmd_bytes = bytes(self.commands[cmd_name])
        
        if DEBUG_LECP6:
            print(cmd_name + ":", cmd_bytes.hex(), end="")
        
        self.ser.write(cmd_bytes)
        # time.sleep(1)
        response = self.ser.read(256)
        
        if DEBUG_LECP6:
            print("\t Response:", response.hex(), "\n")
        
        return response.hex()

    def move_to(self, movement_mode: int = 1, speed: int = 16, position: float = 0.00,
                      acceleration : int = 3000, deceleration : int = 3000,
                      pushing_force: int = 0, trigger_level: int = 0, pushing_speed: int = 16,
                      moving_force: int = 100, area1: float = 0.00, area2: float = 0.00, in_position: float = 0.00):
        """
        Move the actuator to the specified position with the given parameters.
            
        Parameters:
            movement_mode (int): Movement mode (`1` for absolute, `2` for relative).
            speed (int): Speed in mm/s (valid range: 16-500).
            position (float): Target position in mm.
            acceleration (int): Acceleration in mm/s².
            deceleration (int): Deceleration in mm/s².
            pushing_force (int): Pushing force in % (0-100). Use 0 for positioning.
            trigger_level (int): Trigger level in % (0-100).
            pushing_speed (int): Pushing speed in mm/s.
            moving_force (int): Moving force in % (0-300).
            area1 (float): Area output end 1 in mm.
            area2 (float): Area output end 2 in mm.
            in_position (float): In-position tolerance in mm.
        """
        
        DATA = self.get_step_data(movement_mode, speed, position, acceleration, deceleration,
                                  pushing_force, trigger_level, pushing_speed, moving_force, area1, area2, in_position)

        self.send_cmd("SEND DATA", DATA)
        self.send_cmd("START")

        self.wait_till_X_set(1, X4B)

    def get_step_data(self, movement_mode: int = 1, speed: int = 16, position: float = 0.00,
                      acceleration : int = 3000, deceleration : int = 3000,
                      pushing_force: int = 0, trigger_level: int = 0, pushing_speed: int = 16,
                      moving_force: int = 100, area1: float = 0.00, area2: float = 0.00, in_position: float = 0.00):
        """
        Generate data to send with the "SEND DATA" command for actuator movement.

        Parameters:
            movement_mode (int): Movement mode (`1` for absolute, `2` for relative).
            speed (int): Speed in mm/s (valid range: 16-500).
            position (float): Target position in mm.
            acceleration (int): Acceleration in mm/s².
            deceleration (int): Deceleration in mm/s².
            pushing_force (int): Pushing force in % (0-100). Use 0 for positioning.
            trigger_level (int): Trigger level in % (0-100).
            pushing_speed (int): Pushing speed in mm/s.
            moving_force (int): Moving force in % (0-300).
            area1 (float): Area output end 1 in mm.
            area2 (float): Area output end 2 in mm.
            in_position (float): In-position tolerance in mm.

        Returns:
            list of int: A list of bytes representing the command data.
        """

        #convert distances from mm to 0.01 mm as int
        position = int(position * 100)
        area1 = int (area1 * 100)
        area2 = int(area2 * 100)
        in_position = int(in_position * 100)

        DATA = []

        DATA += movement_mode.to_bytes(2, "big")
        DATA += speed.to_bytes(2, "big")
        DATA += position.to_bytes(4, "big")
        
        DATA += acceleration.to_bytes(2, "big")
        DATA += deceleration.to_bytes(2, "big")
        DATA += pushing_force.to_bytes(2, "big")
        DATA += trigger_level.to_bytes(2, "big")

        DATA += pushing_speed.to_bytes(2, "big")
        DATA += moving_force.to_bytes(2, "big")

        DATA += area1.to_bytes(4, "big")
        DATA += area2.to_bytes(4, "big")
        DATA += in_position.to_bytes(4, "big")
        
        
        return DATA

    def process_response(self, cmd_name, response):
        """
        Process the response received from the controller.

        Parameters:
            cmd_name (str or LECP6CMD): The command name associated with the response.
            response (str): The response received from the controller as a hexadecimal string.

        Returns:
            tuple: A tuple containing the response type (LECP6Response) and parsed response data.
        """
        if isinstance(cmd_name, LECP6CMD):
            cmd_name = cmd_name.value

        byte_response = list(bytes.fromhex(response))

        CTRL_ID = byte_response[0]
        FUNC = byte_response[1]

        CRC16 = (byte_response[-1] << 8) | byte_response[-2]


        if FUNC & 0x80:
            ERROR_CODE = byte_response[2]
            return LECP6Response.ERROR, [CTRL_ID, FUNC, ERROR_CODE, CRC16]
        
        CRC = calculate_crc16(byte_response[:-2])

        if CRC != CRC16:
            return LECP6Response.CRC, [CTRL_ID, FUNC, CRC, CRC16]
        
        if self.commands_op[cmd_name] in [RO, RI, RD]:
            DATA_BYTES = byte_response[2]
            DATA = byte_response[3:3 + DATA_BYTES]

            return LECP6Response.VALID, [CTRL_ID, FUNC, DATA_BYTES, DATA, CRC16]

        elif self.commands_op[cmd_name] in [WO, EB, WA, WD]:
            START = byte_response[2:4]
            DATA = byte_response[4:6]

            return LECP6Response.VALID, [CTRL_ID, FUNC, START, DATA, CRC16]
    
    def wait_till_X_set(self, byte_idx, bit, set_bit = True):
        """
        Wait until a specific input bit (X) is set or cleared.

        Parameters:
            byte_idx (int): The index of the byte to check in the input signal.
            bit (int): The bit to check within the specified byte.
            set_bit (bool): Whether to wait for the bit to be set (`True`) or cleared (`False`).
        """
        while True:
            response = self.send_cmd("READ INPUTS")
            response_type, data = self.process_response("READ INPUTS", response)

            if response_type == LECP6Response.ERROR:
                print("Error:", data[2])
            elif response_type == LECP6Response.CRC:
                print("CRC mismatch:", data[-2], data[-1])
            else:
                reg_bytes = data[3]

                if DEBUG_LECP6:
                    print("X4_:")
                    print("FEDCBA98\t76543210")
                    print(f"{reg_bytes[1]:08b}\t{reg_bytes[0]:08b}")
                    print("Byte:", byte_idx, "Bit:", bit, "Flag:", reg_bytes[byte_idx] & bit)

                if bool(reg_bytes[byte_idx] & bit) == set_bit:
                    break
        
        if DEBUG_LECP6:
            print("")


    def __del__(self):
        self.close()

    def close(self):
        """
        Close the serial connection and turn off the servo.
        """
        if self.ser is not None:
            self.send_cmd("SERVO OFF")

            self.ser.close()
            self.ser = None