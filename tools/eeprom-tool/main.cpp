#include <iostream>
#include <stdio.h>
#include <glog/logging.h>
#include <getopt.h>
#include "eepromtoolcontroller.h"
#include <string>

aditof::Status handleWrite(aditof::ConnectionType selectedConnection, const char* path){
   if (path == NULL){
         LOG(ERROR) << "invalid pointer to path";
        return aditof::Status::GENERIC_ERROR;
   }
   printf("writing via %d from %s\n", static_cast<int>(selectedConnection), path);
   return aditof::Status::OK;
}


aditof::Status handleRead(aditof::ConnectionType selectedConnection, const char* path){
   if (path == NULL){
         LOG(ERROR) << "invalid pointer to path";
        return aditof::Status::GENERIC_ERROR;
   }
   printf("reading via %d to %s\n", static_cast<int>(selectedConnection), path);
   return aditof::Status::OK;
}

int main(int argc, char *argv[]){
    //auto controller = std::make_shared<EepromToolController>();
    //std::cout << controller->hasCamera();
    int option;
  
  aditof::ConnectionType selectedConnection;

   while((option = getopt(argc, argv, ":ue:mr:w:")) != -1){ //get option from the getopt() method
      switch(option){
         //For option i, r, l, print that these are options
         case 'u':
            selectedConnection = aditof::ConnectionType::USB;
            printf("Given1 Option: %c\n", option);
           break;
         case 'e':
            selectedConnection = aditof::ConnectionType::ETHERNET;
            printf("Given2 Option: %c with value %s\n", option, optarg);
           break;
           //TO DO: maybe rename this options to Local instead of mipi ?
         case 'm':
            selectedConnection = aditof::ConnectionType::LOCAL;
            printf("Given3 Option: %c\n", option);
            break;
         case 'w': //here f is used for some file name
            handleWrite(selectedConnection, optarg);
            break;
         case 'r': //here f is used for some file name
             handleRead(selectedConnection, optarg);
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
