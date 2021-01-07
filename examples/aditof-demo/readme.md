# aditof-demo evaluation application 

## Overview

The aditof-demo is a cross platform application meant for system evaluation. It can run both on an embedded platform or a host PC by making use of the different conectivity options provided by the SDK.  

For more information about using the application on various platforms see the corresponding user guide below.

| Platfrom | Documentation |
| --------- | ----------- |
| Windows PC | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_windows) |
| Linux PC | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_linux) |
| DragonBoard 410c | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_db410c) |

## Saving data to a file

Currently the filename is limited to the characters `[0-9A-F]`, and the format is binary file.
The structure of the file should look like this:
`height, width, fps, frame1, frame2, ... , frameN`
where `height`, `width` and `fps` are of type `unsigned int` and `frame1` to `frameN` are the actual frames captured of type `uint16_t`.
Reading frames from this files can be done using the code snippet below
```cpp
std::ifstream playbackFile(fileName, std::ios::binary);
unsigned int height = 0;
unsigned int width = 0;
unsigned int fps = 0;

playbackFile.seekg(0, std::ios_base::end);
int fileSize = playbackFile.tellg();
playbackFile.seekg(0, std::ios_base::beg);

playbackFile.read(reinterpret_cast<char *>(&height), sizeof(int));
playbackFile.read(reinterpret_cast<char *>(&width), sizeof(int));
playbackFile.read(reinterpret_cast<char *>(&fps), sizeof(int));

int sizeOfHeader = 3 * sizeof(int);
int sizeOfFrame = sizeof(uint16_t) * height * width;

// Find out how many frames you have in the file
int numberOfFrames = (fileSize - sizeOfHeader) / sizeOfFrame;

FrameDetails frameDetails;
frameDetails.height = height;
frameDetails.width = width;

// Set frame details
frame->setDetails(frameDetails);

Frame frame;
uint16_t *frameDataLocation;
frame->getData(aditof::FrameDataType::FULL_DATA, &frameDataLocation);

// Read one frame from the file. This read can be repeted 
// numberOfFrames times (playbackFile.eof() will return true)
int size = static_cast<int>(sizeof(uint16_t) * width * height);
playbackFile.read(reinterpret_cast<char *>(frameDataLocation),
                                size);
```
Now the frame can be used as normal:
```cpp
uint16_t *data;
status = frame.getData(FrameDataType::DEPTH, &data);
```
