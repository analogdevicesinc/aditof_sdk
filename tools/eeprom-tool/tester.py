from enum import Enum
import subprocess
import os
import filecmp
from time import sleep
from shutil import copyfile

EXEC_NAME = "build/eeprom-tool"
BACKUP_FILE = "backup.bin"
ALTERED_BACKUP_FILE = "altered_backup.bin"
READBACK_FILE = "readback.bin"
BYTE_TO_ALTER_INDEX = 100


class ConnectionType(Enum):
    USB = 1
    ON_TARGET = 2
    NETWORK = 3


class CommandType(Enum):
    READ = 1
    WRITE = 2

# use default eeprom


def run_eeprom_tool(connection_type, command, path, ip=""):
    cmd = "./" + ls 
    connection_type_str = {ConnectionType.USB: "-u",
                           ConnectionType.ON_TARGET: "-m",
                           ConnectionType.NETWORK: "-n"}.get(connection_type, "-")
    ip_str = ip if connection_type == ConnectionType.NETWORK else ""
    cmd_str = {CommandType.READ: "-r",
               CommandType.WRITE: "-w"}.get(command, "-")

    # while (subprocess.run([cmd, connection_type_str, cmd_str, ip_str, path]).returncode != 0):
    #     pass
    completed_process = subprocess.run(
        [cmd, connection_type_str, cmd_str, path])

    print("[TESTER] command `" + ' '.join(completed_process.args) +
          "` returned " + str(completed_process.returncode))
    return completed_process.returncode

# if the writing would not work at all this test would still pass
# TODO modify something before writing


def test_readback(connection_type):
    # backup content
    status = run_eeprom_tool(connection_type, CommandType.READ, BACKUP_FILE)
    if (status == 0):
        print("[TESTER] backed-up content successfully via " + connection_type.name)
    else:
        print("[TESTER] error while backing up content via " +
              connection_type.name)
        return False

    # copy and modify backup file
    copyfile(BACKUP_FILE, ALTERED_BACKUP_FILE)
    with open(ALTERED_BACKUP_FILE, 'r+b') as f:
        f.seek(BYTE_TO_ALTER_INDEX)
        c = f.read()[0]
        f.seek(BYTE_TO_ALTER_INDEX)
        f.write(bytes(c + 1))

    # write the same values
    status = run_eeprom_tool(
        connection_type, CommandType.WRITE, ALTERED_BACKUP_FILE)
    if (status == 0):
        print("[TESTER] wrote content successfully via " + connection_type.name)
    else:
        print("[TESTER] error while writing content via " + connection_type.name)
        return False

    # readback
    status = run_eeprom_tool(connection_type, CommandType.READ, READBACK_FILE)
    if (status == 0):
        print("[TESTER] readback successfully via " + connection_type.name)
    else:
        print("[TESTER] error while reading back content via " +
              connection_type.name)
        return False

    # restore file
    status = run_eeprom_tool(connection_type, CommandType.WRITE, BACKUP_FILE)
    if (status == 0):
        print("[TESTER] wrote content successfully via " + connection_type.name)
    else:
        print("[TESTER] error while writing content via " + connection_type.name)
        return False

    match = filecmp.cmp(ALTERED_BACKUP_FILE, READBACK_FILE)

    print("[TESTER] readback test on " + connection_type.name +
          " has " + "passed" if match else "failed")

    # cleanup
    os.remove(BACKUP_FILE)
    os.remove(ALTERED_BACKUP_FILE)
    os.remove(READBACK_FILE)

    return match


def main():
    test_readback(ConnectionType.ON_TARGET)
    test_readback(ConnectionType.USB)


if __name__ == "__main__":
    main()
    pass
