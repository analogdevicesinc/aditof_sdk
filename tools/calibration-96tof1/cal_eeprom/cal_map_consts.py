'''
Predefined Hashmap key Defination
---------------------------------
'''
# Dictionary for modes
mode_dict = {'near': 0, 'mid': 1, 'far': 2}

# Hashmap key for packet type
HEADER = 0
CAMERA_INTRINSIC = 1
NEAR_CAL = 2
NEAR_LF = 3
MID_CAL = 4
MID_LF = 5
FAR_CAL = 6
FAR_LF = 7


# Hashmap key for common parameters
EEPROM_VERSION = 1
CAL_SER_NUM = 2
CAL_DATE = 3
CHECKSUM = 4


# Hashmap key for Header Parameters
TOTAL_SIZE = 5
NUMBER_OF_MODES = 6
MODULE_TYPE = 11  # Value 2: BroadMarket/1: PICO/0 : ADI EVAL
AFE_TYPE = 13  # Value 0: ADDI9033 / 1:ADDI9043/ 2: ADDI9050
SENSOR_TYPE = 14  # Value 0 : Panasonic VGA / 1 : Panasonic QVGA
LASER_TYPE = 16  # Value 0 : Princeton VCSEL/ 1 : Heptagon VCSEL...

# Hashmap key for Camera Intrinsic
INTRINSIC = 5
DISTORTION_COEFFICIENTS = 6

# Hashmap for linear correct
ISATG_PROJECT_VERSION = 5
CALIBRATION_SOFTWARE_VERSION = 6
CALIBRATION_TYPE = 7  # Value 0 Sweep, 1: Rail, 2: Faceplant
CALIBRATION_MODE = 8  # Value 0:Near 1, 1 : Mid, 2 :Far
PULSE_COUNT = 11
NO_OF_LASERS = 12
LINEAR_CORRECT_OFFSET = 22
LINEAR_CORRECT_XPWR = 23

# Hashmap for load files
ADDR_DATA_LIST = 5

# Indices for PARAM STRUCT
SIZE = 0
VALUE = 1
