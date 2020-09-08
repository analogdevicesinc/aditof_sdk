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

Status handleWrite(ConnectionType connectionType, const char* path){
   if (path == NULL){
        LOG(ERROR) << "invalid pointer to path";
        return Status::GENERIC_ERROR;
   }
   aditof::Status status;
   auto controller = std::make_shared<EepromToolController>();
   
   status = controller->setConnection(connectionType);
   if (status != aditof::Status::OK){
      LOG(ERROR) << "cannot set connection";
      return status;
   }

   controller->writeFileToEeprom(path);

   return Status::OK;
}

Status handleRead(ConnectionType connectionType, const char* path){
   if (path == NULL){
         LOG(ERROR) << "invalid pointer to path";
        return Status::GENERIC_ERROR;
   }
   aditof::Status status;
   auto controller = std::make_shared<EepromToolController>();
   
   status = controller->setConnection(connectionType);
   if (status != aditof::Status::OK){
      LOG(ERROR) << "cannot set connection";
      return status;
   }

   controller->readEepromToFile(path);
   
   return Status::OK;
}

int main(int argc, char *argv[]){
    //std::cout << controller->hasCamera();
   int option;
   std::string path; 

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
         case 'w': 
            handleWrite(connectionType, optarg);
     //       path = string(optarg);
            break;
         case 'r': 
       //     path = string(optarg);
            handleRead(connectionType, optarg);
         //   printf("got path %s\n", path.c_str());
            break;
         case ':':
            printf("option needs a value\n");
            break;
         case '?': //used for some unknown options
            printf("unknown option: %c\n", optopt);
            break;
      }
   }

  return 0;
}
