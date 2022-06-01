/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef CUDA_UTILS
#define CUDA_UTILS

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <string>
#include <vector>

#define THREAD_PER_BLOCK 1024


class cudaOnTarget {
  private:
    //correction cahes
    double *m_geometry_cache_d;
    double *m_distortion_cache_d;
    uint16_t *m_depth_cache_d;

    // double *m_geometry_cache;
    // double *m_distortion_cache;
    // uint16_t *m_depth_cache;

    //frames and parameters
    uint16_t *m_frame_d;
    uint16_t *m_frame;
    double *m_parameters_d;
    double *m_parameters;

  public:
    void buildGeometryCorrectionCache();
    void buildDistortionCorrectionCache();
    void buildDepthCorrectionCache();

    void applyGeometryCorrection();
    void applyDistortionCorrection();
    void applyDepthCorrection();

    void cpyFrameToGPU(uint16_t *frame);
    void cpyFrameFromGPU(uint16_t *frame);
    void printFrameFromGPU();

    void setParameters(double* parameters);
    //Order of parameters: 
    // double width, double height, double fx, double fy,
    //                    double cx, double cy, double k1, double k2, double k3,
    //                    double x0, double y0, double gain, double offset,
    //                    double pixelMaxValue, double range
    void freeAll();
};

#endif // CUDA_UTILS
