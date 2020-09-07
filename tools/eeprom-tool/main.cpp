#include <iostream>
#include <stdio.h>
#include <glog/logging.h>
#include <getopt.h>
#include <string>
#include <aditof/eeprom_factory.h>
#include <aditof/device_construction_data.h>
#include "eepromtoolcontroller.h"
#include <iterator>
#include <algorithm>

using namespace aditof;
using namespace std;

Status handleWrite(unique_ptr<EepromInterface>& eeprom, const char* path){
   if (path == NULL){
         LOG(ERROR) << "invalid pointer to path";
        return Status::GENERIC_ERROR;
   }
   string name;
   eeprom->getName(name);
   printf("writing via %s from %s\n", name.c_str(), path);
   return Status::OK;
}

Status handleRead(unique_ptr<EepromInterface>& eeprom, const char* path){
   if (path == NULL){
         LOG(ERROR) << "invalid pointer to path";
        return Status::GENERIC_ERROR;
   }
   string name;




   eeprom->getName(name);
   printf("reading via %s to %s\n", name.c_str(), path);
   return Status::OK;
}

int main(int argc, char *argv[]){
    auto controller = std::make_shared<EepromToolController>();
    //std::cout << controller->hasCamera();
    int option;

   ConnectionType connectionType;
  unique_ptr<EepromInterface> eeprom;

   while((option = getopt(argc, argv, ":ue:mr:w:")) != -1){ //get option from the getopt() method
      switch(option){
         //For option i, r, l, print that these are options
         case 'u':
            connectionType = ConnectionType::USB;
            printf("Given1 Option: %c\n", option);
           break;
         case 'e':
            connectionType = ConnectionType::ETHERNET;
            printf("Given2 Option: %c with value %s\n", option, optarg);
           break;
           //TO DO: maybe rename this options to Local instead of mipi ?
         case 'm':
            connectionType = ConnectionType::LOCAL;
            printf("Given3 Option: %c\n", option);
            break;
         case 'w': //here f is used for some file name
            //handleWrite(eeprom, optarg);
            break;
         case 'r': //here f is used for some file name
             //handleRead(eeprom, optarg);
            break;
         case ':':
            printf("option needs a value\n");
            break;
         case '?': //used for some unknown options
            printf("unknown option: %c\n", optopt);
            break;
      }
   }

   printf("found connection %d\n", controller->setConnection(connectionType) == aditof::Status::OK);

  return 0;
}
