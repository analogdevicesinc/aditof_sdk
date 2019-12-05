The python based calibration software is supported on the DragonBoard 410c Broad Market Kit as well as Windows 10. It is highly recommended to run the calibration software on the Dragonboard platform.

# Dragonboard Install
The basic setup of the Broad Market Kit is described [here](https://github.com/analogdevicesinc/aditof_sdk).

Clone this SDCARD image to a 16GB SDCARD and put the SD Card slot of the Dragonboard and power on the board.

## Run Sweep Calibtation For A Given Mode

Enable the python environment

``` 
cd ~/workspace/calibration
source py36tofcalib/bin/activate
```

The parameters are defaulted in the calibrate_single_mode.py file

``` 
python calibrate_single_mode.py --help
```

Example Run

By default the calibration is done using timing sweep
```
python calibrate_single_mode.py config/BM_kit/Near/sweep_config_near.json

```

To change the default parameters pass arguments on the command line or modify the sweep_config_xxx.json file. For e.g. to run a rail sweep and verification modify the "calib_type" and "verification_type" params to "Rail".

### Write firmware to eeprom
To write a specific firmware to eeprom:
Modify cal_replace.json for appropriate firmware paths.

```
cd scripts
sudo ../../py36tofcalib/bin/python eeprom_replace_cal.py cal_replace.json

```

## Controlling logger output
The logger.json allows you configure the logger output.You can set the appropriate logging level. Here's good discription: [logging levels](https://docs.python.org/3.6/library/logging.html#levels)

## Saved Results
```
The scripts generate a unique_id by default. The generated unique_id is displayed on the console output when running the calibration. The results are saved in saved_results/unique_id/latest and also archived in saved_results/unique_id/archive/<timestamped_dir>. The following data is saved

 * csv's: The linear_offset table and the pre and post calibrated data statistics are saved as csv's
 * input_config.json: All the configurations/parameters used for the experiment
 * lf_files: All the firmware files used for the experiment

```

# Windows Install
## Dependencies

* Install the latest version of [Miniconda for Python 3.7](https://conda.io/miniconda.html) 

## Install
```
conda env create -f environment.yml 
source activate py36tofcalib
```

### Troubleshooting Conda Environment Setup
 * Error: Permission denied 13
   * Possible workarounds:
     * Install Anaconda/Miniconda for User instead of all 'All Users'
     * If that doesn't help try install with Admin privileges: run Anaconda prompt or cmd prompt as admin.
     * Try updating conda:
```
conda update --all
```

## Setup ADI TOF Python wrappers (bindings)
Follow the steps in the following repo to install the tof_device python module in your environment:
https://github.com/adi-sdg/tof_sdk/tree/master/IO_Library/python


## Setup Environment for Sweep Calibration

* Plug ADI TOF camera module to a host (Windows, Linux or OSX)
* Make sure the ADI camera is recognized by the host
* Set a target - a plain white board within 20cm to 200 cm in front of the camera - the distance between the target and the edge of camera lens is the target_distance 

## Run Sweep Calibtation For A Given Rang or Mode

The parameters are defaulted in the calibrate_single_mode.py file

``` 
python calibrate_single_mode.py --help
```

Example Run

```
python calibrate_single_mode.py config/sweep_config.json --firmware-path config/ADDI9043

```

Or add command line arguments - note that the values specified on the command line will overwrite the defaults in the sweep config json file


```
python calibrate_single_mode.py config/sweep_config.json --firmware-path config/ADDI9043 --min-dist 0 --max-dist 2000

```

## Run and Verify Sweep

```
python calibrate_single_mode.py config/sweep_config.json --firmware-path config/ADDI9043 --min-dist 0 --max-dist 2000 --verify-sweep
```

## Explore results
```
jupyter notebook
```

The browser will open a Jupyter Home showing all the files in the current working directory. Click on the explore_sweep_results.ipynb for a sample example of loading the csv files and exploring the results. You may have to change the path of the results_path variable.

## Fully Pipelined Setup for the Broad Market Kit

```
python run_pipeline.py

  * Prompts the user for a station selection. Type the full name of the station as displayed on the menu.
  * The program will make you confirm your selection by hitting ENTER.
  * After calibration has finished for one station, continue to other stations, enter exit to exit the program.

Follow the complete setup instructions in docs/CM_Manual.md
```

