#ifndef UTILS_LINUX_H
#define UTILS_LINUX_H

#include <errno.h>
#include <sys/ioctl.h>

#ifdef DEBUG_USB
#include <iostream>
#include <linux/uvcvideo.h>
#include <string.h>
#endif

static int xioctl(int fh, unsigned long request, void *arg) {
    int r;

    do {
        r = ioctl(fh, request, arg);
        // printf("Error=%d\n",errno);
    } while (-1 == r && (EINTR == errno || EIO == errno) && errno != 0);

#ifdef DEBUG_USB
    if (request == (int)UVCIOC_CTRL_QUERY) {
        static int count = 0;
        struct uvc_xu_control_query *cq = (struct uvc_xu_control_query *)arg;
        count++;
        printf("%d Calling IOCTL with bRequest %02x, wValue %04x, wIndex %04x, "
               "wLength %04x\n",
               count, cq->query, cq->selector, cq->unit, cq->size);
        if (r != 0) {
            printf("Return values: %d \n", r);
            printf("IOCTL failed, error num: %d, %s\n", errno, strerror(errno));
        }
    }
#endif
    return r;
}

#endif // UTILS_LINUX_H
