#ifndef ADITOF_UTILS_H
#define ADITOF_UTILS_H

#include <aditof/camera.h>

std::shared_ptr<aditof::Camera> initCameraEthernet(int argc, char **argv);

void applyNoiseReduction(const std::shared_ptr<aditof::Camera> &camera,
                         int argc, char **argv);
uint16_t *getNewFrame(const std::shared_ptr<aditof::Camera> &camera,
                      aditof::Frame *frame);

aditof::IntrinsicParameters
getIntrinsics(const std::shared_ptr<aditof::Camera> &camera);
int getRangeMax(const std::shared_ptr<aditof::Camera> &camera);
int getRangeMin(const std::shared_ptr<aditof::Camera> &camera);

#endif // ADITOF_UTILS_H
