#include "eeprom_tool.h"

#include <iostream>
#include <stdio.h>
#include <glog/logging.h>
#include <getopt.h>
#include <string>
#include <iterator>
#include <algorithm>

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

Status parseArguments(int argc, char *argv[], CLIArguments& cliArguments){
   int option;

   while((option = getopt(argc, argv, ":ue:mn:r:w:l")) != -1){ //get option from the getopt() method
      switch(option){
         case 'u':
            cliArguments.connectionType = ConnectionType::USB;
            cliArguments.isConnectionSpecifed = true;
            break;
         case 'e':
            cliArguments.connectionType = ConnectionType::ETHERNET;
            cliArguments.ip = string(optarg);
            cliArguments.isConnectionSpecifed = true;
            break;
         case 'm':
            cliArguments.connectionType = ConnectionType::LOCAL;
            cliArguments.isConnectionSpecifed = true;
            break;
         case 'n':
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
         case ':':
            printf("option needs a value\n");
            return Status::INVALID_ARGUMENT;
            break;
         case '?': //used for some unknown options
            printf("unknown option: %c\n", optopt);
            break;
      }
   }

   return Status::OK;
}

int main(int argc, char *argv[]){
   Status status;
   CLIArguments cliArguments;
   auto controller = std::make_shared<EepromTool>();

   status = parseArguments(argc, argv, cliArguments);
   if (status != aditof::Status::OK){
      LOG(ERROR) << "cannot parse CLI arguments";
      return 1;
   }

   if (cliArguments.isConnectionSpecifed){
      status = controller->setConnection(cliArguments.connectionType, cliArguments.ip, cliArguments.eepromName);
   }
   else if (aditof::Status::OK == (status = controller->setConnection(ConnectionType::LOCAL, cliArguments.ip, cliArguments.eepromName))){
      LOG(INFO) << "setting connection via MIPI";
   }
   else if (aditof::Status::OK == (status = controller->setConnection(ConnectionType::USB, cliArguments.ip, cliArguments.eepromName))){
      LOG(INFO) << "setting connection via USB";
   }

   if (status != aditof::Status::OK){
      LOG(ERROR) << "cannot set connection";
      return 1;
   }

   switch (cliArguments.actionType)
   {
   case READ:
         status = controller->readEepromToFile(cliArguments.path.c_str());
      break;
   case WRITE:
         status = controller->writeFileToEeprom(cliArguments.path.c_str());
      break;
   case LIST_EEPROMS:
         status = controller->listEeproms();
      break;
   default:
      break;
   }

   if (status != Status::OK){
      return 1;
   }

  return 0;
}
