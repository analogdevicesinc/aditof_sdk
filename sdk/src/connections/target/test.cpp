#include <iostream>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>



static int xioctl(int fh, unsigned int request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno && errno != 0);

    return r;
}

int main(int argv, char* argc[])
{
    struct v4l2_input in;
    int fh = open(argc[1], O_RDWR);
    std::cout<<fh << ". "<<xioctl(fh,VIDIOC_ENUMINPUT, &in)<<'\n'<<in.type ;


}