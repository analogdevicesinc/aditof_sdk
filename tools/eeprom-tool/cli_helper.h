#ifndef CLI_HELPER_H
#define CLI_HELPER_H

#include <getopt.h>
#include <iostream>
#include <stdio.h>
#include <string>

#include <aditof/status_definitions.h>
#include <aditof/device_interface.h>
#include <aditof/connections.h>

using namespace aditof;
using namespace std;

enum ActionType {
   WRITE,
   READ,
   LIST_EEPROMS,
   UNKNOWN
};

typedef struct {
   string path;
   string ip = "0.0.0.0";
   string eepromName = "";
   ConnectionType connectionType = ConnectionType::LOCAL;
   ActionType actionType = UNKNOWN;
   bool isConnectionSpecifed = false;
} CLIArguments;



void printHelpMessage();
Status parseArguments(int argc, char *argv[], CLIArguments& cliArguments);


#endif