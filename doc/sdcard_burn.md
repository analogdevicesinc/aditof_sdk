# Writing the SD card image onto the SD card

The size of the SD card that is going to be used to write the image to, must be equal or greater than the size of the image.
Writing the image on the SD card will take about 15 to 20 minutes for a 16GB image, depending on the transfer speed of the system being used. 

## Linux
* Make sure that xz-utils are installed on your machine
    ~~~sh
    sudo apt-get install xz-utils
    ~~~
* Unpack the .tar.xz archive
    ~~~sh
    tar -xf name-of-the-image.tar.xz
    ~~~
* Write the image on the SD card
    ~~~sh
    dd if=name-of-the-image.img of=/dev/xxx bs=4M
    sync
    ~~~
    
Note 1: The following command can be used to help identifying the device that needs to be specified for the argument **of=/dev/xxx**:
~~~sh
    lsblk
~~~
Note 2: It is very likely that the **dd** command will have to be executed with **sudo** privileges:
~~~sh
    sudo dd if=name-of-the-image.img of=/dev/xxx bs=4M
~~~


## Windows
* Unpack the .tar.xz archive using [7-Zip](https://www.7-zip.org/)
* Install the [balenaEtcher](https://www.balena.io/etcher/) app
* Write the .img file to the SD card using the balenaEtcher app
