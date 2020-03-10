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
