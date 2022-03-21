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
#include "eeprom_tool.h"

#include <algorithm>
#ifndef JS_BINDINGS
#include <glog/logging.h>
#else
#include <aditof/log_cout.h>
#endif
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <string>

using namespace aditof;
using namespace std;

int main(int argc, char *argv[]) {
    Status status;
    CLIArguments cliArguments;
    auto controller = EepromTool();

    status = parseArguments(argc, argv, cliArguments);
    if (status != aditof::Status::OK) {
        return 1;
    }

    //try to set a connection of the specied type
    if (cliArguments.isConnectionSpecifed) {
        status =
            controller.setConnection(cliArguments.connectionType,
                                     cliArguments.ip, cliArguments.eepromName);
    }
    //if none is specified try local and usb
    else if (aditof::Status::OK ==
             (status = controller.setConnection(ConnectionType::ON_TARGET,
                                                cliArguments.ip,
                                                cliArguments.eepromName))) {
    } else if (aditof::Status::OK == (status = controller.setConnection(
                                          ConnectionType::USB, cliArguments.ip,
                                          cliArguments.eepromName))) {
    }

    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Cannot set connection";
        return 1;
    }

    switch (cliArguments.actionType) {
    case READ:
        status = controller.readEepromToFile(cliArguments.path.c_str());
        break;
    case WRITE:
        status = controller.writeFileToEeprom(cliArguments.path.c_str());
        break;
    case LIST_EEPROMS:
        status = controller.listEeproms();
        break;
    default:
        break;
    }

    if (status != Status::OK) {
        return 1;
    }

    return 0;
}
