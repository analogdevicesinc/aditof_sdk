#include "eeprom_tool.h"
#include "cli_helper.h"

#include <iostream>
#include <stdio.h>
#include <glog/logging.h>
#include <string>
#include <iterator>
#include <algorithm>

using namespace aditof;
using namespace std;

int main(int argc, char *argv[]){
   Status status;
   CLIArguments cliArguments;
   auto controller = std::make_shared<EepromTool>();

   status = parseArguments(argc, argv, cliArguments);
   if (status != aditof::Status::OK){
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
