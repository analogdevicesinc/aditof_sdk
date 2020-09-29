import json
import logging
import logging.config
import sys
import click
import os

mode_folder_names = ["Far", "Mid", "Near"]
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
                     ]


def setup_logging():
    with open('logger.json', 'r') as f:
        config = json.load(f)
        logging.config.dictConfig(config)


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
    valid_subfolders = [s for s in subfolders if s.name in mode_folder_names]
    # check the content of the folders
    valid_subfolders = [
        s for s in valid_subfolders if check_folder_integrity(s.path)]

    print("Will generate binary file for the following modes:\n\t" +
          "\n\t".join([s.name for s in valid_subfolders]))

    return [s.path for s in valid_subfolders]


@click.command()
@click.argument('source-folder-path', type=click.Path(exists=True))
@click.argument('bin-file-name', type=click.File('wb'))
def run_gen(source_folder_path, **kwargs):
    logger = logging.getLogger(__name__)
    check_folder_structure(source_folder_path)


if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)
    run_gen()
