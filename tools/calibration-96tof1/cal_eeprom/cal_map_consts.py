'''
Predefined Hashmap key Defination
---------------------------------
'''
# Dictionary for modes
mode_dict = {'near': 0, 'mid': 1, 'far': 2}


# helper functions. cannot move to utils because of circular dependecy
def get_cal_key(mode):
    return mode_dict[mode] * 2 + 2


def get_lf_key(mode):
    return mode_dict[mode] * 2 + 3


# Hashmap key for packet type
HEADER = 0
CAMERA_INTRINSIC = 1
NEAR_CAL = get_cal_key("near")
NEAR_LF = get_lf_key("near")
MID_CAL = get_cal_key("mid")
MID_LF = get_lf_key("mid")
FAR_CAL = get_cal_key("far")
FAR_LF = get_lf_key("far")


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

LINEAR_CAL_FILE_NAME = "linear_cal.json"
INTRINSIC_FILE_NAME = "camera_intrinsic.json"

source_file_names = ["1_AFEstartup_silicon.lf",
                     "2_LDstartup.lf",
                     "3_MIPIstartup_ADDI9033_CYP.lf",
                     "4_Driver_enable.lf",
                     "5_mn34906bl_addi9033_HPT_data.lf",
                     "6_mn34906bl_addi9033_data.lf",
                     "7_mn34906bl_addi9033.lf",
                     "8_LoopNumAddrList.lf",
                     "9_RepeatNumAddrList.lf",
                     "10_TGstartup_ADILD.lf",
                     "11_XV_mux.lf",
                     "12_TOF_ProcCtrl.lf",
                     "13_Mode_Start.lf",
                     #  LINEAR_CAL_FILE_NAME,
                     ]
