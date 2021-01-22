/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "cli_helper.h"

const char *helpString = R"V0G0N(
   Usage: eeprom-tool [(-u | -m | -e <ip>)] [-n <eeprom_name>]  (-h | -l | -w <file_path> | -r <file_path>) 
   
   Reads or writes the contents of the eeprom memory to or from a file. The connection interface may be mipi, local or network.
   If none is specified the tool tries usb and local.
   Also, if multiple memories are available the target should be specified by name. All memories available can be listed.
   
   Connection interfaces:
   -u, --usb      connect to the eeprom memory through usb.
   -m, --mipi     connect to the eeprom memory through mipi (local connection)
   -n, --network  connect to the eeprom memory through network, the <ip> of the target must be specified

   -e, --eeprom   specifies the name of eeprom on the target.

   -r, --read     reads the contents of the eeprom memory and writes it into the specified file
   -w, --write    reads the contents of the specified file and writes is into the eeprom memory 
   -l, --list     list all the available memories
   -h, --help     prints this message 
   
   )V0G0N";

struct option longopts[] = {{"usb", no_argument, NULL, 'u'},
                            {"network", required_argument, NULL, 'n'},
                            {"mipi", no_argument, NULL, 'm'},
                            {"eeprom", required_argument, NULL, 'e'},
                            {"write", required_argument, NULL, 'w'},
                            {"read", required_argument, NULL, 'r'},
                            {"list", no_argument, NULL, 'l'},
                            {"help", no_argument, NULL, 'h'},
                            {0, 0, 0, 0}};

void printHelpMessage() { printf("%s", helpString); }

Status parseArguments(int argc, char *argv[], CLIArguments &cliArguments) {
    int option;

    while (
        (option = getopt_long(argc, argv, ":un:me:r:w:lh", longopts, NULL)) !=
        -1) { //get option from the getopt() method
        switch (option) {
        case 'u':
            cliArguments.connectionType = ConnectionType::USB;
            cliArguments.isConnectionSpecifed = true;
            break;
        case 'n':
            cliArguments.connectionType = ConnectionType::NETWORK;
            cliArguments.ip = string(optarg);
            printf("%s\n", cliArguments.ip.c_str());
            cliArguments.isConnectionSpecifed = true;
            break;
        case 'm':
            cliArguments.connectionType = ConnectionType::ON_TARGET;
            cliArguments.isConnectionSpecifed = true;
            break;
        case 'e':
            cliArguments.eepromName = string(optarg);
            break;
        case 'w':
            cliArguments.actionType = WRITE;
            cliArguments.path = string(optarg);
            break;
        case 'r':
            cliArguments.actionType = READ;
            cliArguments.path = string(optarg);
            break;
        case 'l':
            cliArguments.actionType = LIST_EEPROMS;
            break;
        case 'h':
            printHelpMessage();
            return Status::INVALID_ARGUMENT;
            break;
        case ':':
            printf("option needs a value\n");
            printHelpMessage();
            return Status::INVALID_ARGUMENT;
            break;
        case '?': //used for some unknown options
            printf("unknown option: %c\n", optopt);
            printHelpMessage();
            return Status::INVALID_ARGUMENT;
            break;
        }
    }

    return Status::OK;
}
