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
#ifndef DRAGONBOARD_DEFINITIONS_H
#define DRAGONBOARD_DEFINITIONS_H

#define GPIOCHIP2_BASE 504   /* Check file /sys/kernerl/debug/gpio */
#define GPIOCHIP2_PIN_MPP4 3 /* The 4th multipurpose pin of pm8916 */

#define BUTTON1_GPIO (GPIOCHIP2_BASE + GPIOCHIP2_PIN_MPP4)
#define BUTTON2_GPIO 24
#define LED1_GPIO 17
#define LED2_GPIO 19

#define NETWORK_SERVER "aditof-server"

#define NETWORK_SERVER_START_COMMAND                                           \
    "/home/linaro/workspace/github/aditof_sdk/build/apps/"                     \
    "server/" NETWORK_SERVER

#define UVC_APP "uvc-gadget"
#define UVC_APP_START_SCRIPT "config_pipe.sh"
#define UVC_APP_START_COMMAND                                                  \
    "/home/linaro/workspace/github/aditof_sdk/build/apps/"                     \
    "uvc-app/" UVC_APP_START_SCRIPT

#endif // DRAGONBOARD_DEFINITIONS_H
