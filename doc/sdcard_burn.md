# Writting the SD card image onto the SD card

The size SD card used to write the image to must be equal or greater than the size of the image.
Writing the image on the SD card will take about 15 to 20 minutes for a 16GB image, depending on the transfer speed of the system being used. 

## Linux
* Make sure that xz-utils are installed on your machine
    ~~~sh
    sudo apt-get install xz-utils
    ~~~
* Unpack the .tar.xz archive
    ~~~sh
    tar xf xxx.tar.gz
    ~~~
* Write the image on the SD card
    ~~~sh
    dd if=xxx.img of=/dev/xxx bs=4M
    sync
    ~~~
    
Note: The **lsblk** command can be used to help identifying the device to be specified for the argument: **of=/dev/xxx**.

## Windows
* Unpack the .tar.xz archive using [7-Zip](https://www.7-zip.org/)
* Install the [balenaEtcher](https://www.balena.io/etcher/) app
* Write the .img file to the SD card using the balenaEtcher app
