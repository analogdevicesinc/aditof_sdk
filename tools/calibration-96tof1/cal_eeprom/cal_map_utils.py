import os
from cal_map import cal_map
from cal_map_consts import *


def flatten_cal_map(cal):
    lst = []
    for key, list_params in cal.items():
        size, nested_dict = list_params
        lst.append(key)
        lst.append(size)
        for nested_key, nested_value in nested_dict.items():
            param_size, param_value = nested_value
            lst.append(param_size)
            for i in range(int(param_size/4)):
                lst.append(param_value[i])
    return lst


def compare_map(cal1, cal2):
    lst1 = flatten_cal_map(cal1)
    lst2 = flatten_cal_map(cal2)
    ret = True
    for i in range(len(lst1)):
        if(lst1[i]-lst2[i] > 0.2):
            ret = False
    return ret


def check_folder_integrity(root_path):
    found_files = [f.name for f in os.scandir(root_path) if f.is_file()]
    missing_files = [
        f for f in source_file_names if f not in found_files]
    if (len(missing_files)):
        print("Missing the following files:\n\t" +
              "\n\t".join([f for f in missing_files]) +
              " in " + root_path)
    return len(missing_files) == 0


def check_folder_structure(root_path):
    subfolders = [f for f in os.scandir(root_path) if f.is_dir()]
    # check the name of the folders
    valid_subfolders = [
        s for s in subfolders if s.name.lower() in mode_dict.keys()]
    # check the content of the folders
    valid_subfolders = [
        s for s in valid_subfolders if check_folder_integrity(s.path)]

    print("Will generate binary file for the following modes:\n\t" +
          "\n\t".join([s.name for s in valid_subfolders]))

    return [s.path for s in valid_subfolders]


'''
Test function to:
    Generate default map
    Write map to binary file
    Read map back from binary file
    Add load files to map
    Display the map
---------------------------------
'''


def test_cal_eeprom():
    cal1 = cal_map()
    cal1.init_default_cal_map()
    print("Calibration map written to Bin File")
    cal1.display_cal_map()
    cal1.save_cal_map("calibration_map.bin")

    cal2 = cal_map()
    cal2.read_cal_map("calibration_map.bin")
    print("\n\nCalibration map read back from Bin File")
    cal2.display_cal_map()
    input("Press Enter to continue...")

    cal2.add_json_to_map(NEAR_CAL, "./linear_cal.json")
    print("\n\nCalibration map after adding linear_cal.json to map")
    cal2.display_cal_map()
    input("Press Enter to continue...")

    cal2.add_load_files_to_map(NEAR_LF, "../config/ADDI9043/")
    print("\n\nCalibration map after adding load files to map")
    cal2.display_cal_map()
    print("\n\nSaving to 'calibration_map.bin'")
    cal2.save_cal_map("caibration_map.bin")
    input("Press Enter to continue...")

    # Open the ADI TOF Camera
    system = tof.System()
    cam_handle = device.open_device2(system)
    eeproms = []
    cam_handle.getEeproms(eeproms)
    eeprom = eeproms[0]

    print("\n\nWriting to EEPROM")
    cal2.write_eeprom_cal_map(eeprom)

    print("\n\nReading from EEPROM")
    cal3 = cal_map()
    cal3.read_eeprom_cal_map(eeprom)
    cal3.save_cal_map("eeprom_read_map.bin")
    with open("eeprom_read_map.json", 'w') as f:
        f.write(str(cal3.calibration_map))
    f.close()
    cal3.display_cal_map()
    input("Press Enter to continue...")

    cal3.replace_eeprom_mode(
        'near', "./config/BM_Kit/Near/linear_cal.json", "./config/BM_Kit/Near/")
    with open("eeprom_read_map_modified.json", 'w') as f:
        f.write(str(cal1.calibration_map))
    f.close()
