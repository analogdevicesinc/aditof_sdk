## This project builds the hash map from Calibration data and/or AFE firmware. The Hash map is for storing calibration data in EEPROM.

### Run C++
1. Requires C/C++ Compiler
    * Linux - Compiler included in environment [Calibration Environment Setup](../Readme.md)
    * Windows
        * Download and install [Visual Studio](https://visualstudio.microsoft.com/downloads/)
                                    Or
        * Install [MinGW](https://osdn.net/projects/mingw/releases/) 
2. Install [CMake](https://cmake.org/install/)
    * Linux
    ```
    sudo apt install cmake
    ```

3. Create a folder called build in ./C++/ and open folder ./C++/build/
    ```
    cd .\C++\
    mkdir build
    cd build
    ```

4. Compile the project
    ``` 
    cmake .. 
    ```

    * For Linux
    ```
    make .
    ```

    * For Windows 
        * Debug
        ```    
        cmake --build ./ --config Debug 
        ```
        * Release
        ```  
        cmake --build ./ --config Release 
        ```


5. Run the Program
    
    * Linux
    ```
    .\CalEEPROM
    ```

    * Windows
        * Debug
        ``` 
        cd ./Debug/
        .\CalEEPROM.exe
        ```
        * Release
        ``` 
        cd ./Release/
        .\CalEEPROM.exe
        ```

