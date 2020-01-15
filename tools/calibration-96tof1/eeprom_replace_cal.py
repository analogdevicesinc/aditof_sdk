import aditofpython as tof
import tof_calib.device as device
import click
import json
import logging
import logging.config
import uuid

import cal_eeprom.cal_eeprom as ce


def setup_logging():
    with open('logger.json', 'r') as f:
       config = json.load(f)
       logging.config.dictConfig(config)
       


@click.command()
@click.argument('config-json', type=click.STRING)
def run_eeprom_replace_cal(config_json, **kwargs):
    '''
    Replace the calibration data for a single mode in an ADI TOF module

    SWEEP_CONFIG_JSON: Specify the default sweep config json file. The default options can be overriden by explictly specifying the options on the command line
    '''
    logger = logging.getLogger(__name__)
    
    config_dict = {}
    if(config_json is not None):
        with open(config_json) as f:
            config_dict = json.load(f)

    for key,value in kwargs.items():
        if value is not None:
            config_dict[key] = kwargs[key]
        if key in config_dict:
            if config_dict[key] is None:
                print('Need to specify %s either as a command line option or in the sweep config json file')
                raise SystemExit
    
    # Retrieve parameters
    cal_map_path = config_dict['cal_map_path']
    firmware_path = config_dict['firmware_path']
    mode = config_dict['mode']

    # Open the ADI TOF Camera
    system = tof.System()
    status = system.initialize()
    print("system.initialize()", status)
    cam_handle = device.open_device2(system)
    dev_handle = cam_handle.getDevice()

    # Read the cal map from the camera and store it locally
    cal = ce.cal_map()
    cal.read_eeprom_cal_map(dev_handle)
    
    save_path = cal_map_path + "/eeprom_read_map.bin"
    logger.info("Save the original cal map to .bin and .json files, path: %s", save_path)
    cal.save_cal_map(save_path)
    save_path = cal_map_path + "/eeprom_read_map.json"
    with open (save_path, 'w') as f:
        f.write(str(cal.calibration_map))
    f.close()

    #cal.display_cal_map()
    #input("Press Enter to continue...") 

    # Replace a single mode's cal data
    logger.info("Replace mode %s cal data from path: %s", mode, save_path)
    cal.replace_eeprom_mode(mode, firmware_path + "/linear_cal.json", firmware_path)
    save_path = cal_map_path + "/eeprom_read_map_modified.json"
    with open (save_path, 'w') as f:
        f.write(str(cal.calibration_map))
    f.close()

    # Write the cal map back to the camera's EEPROM
    cal.write_eeprom_cal_map(dev_handle)
    logger.info("Write back to EEPROM complete")

if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)
    run_eeprom_replace_cal()

