#include "gpios.h"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <syslog.h>
#include <unistd.h>

#include <stdio.h>

#define BUTTON1_GPIO 28
#define BUTTON2_GPIO 24
#define LED1_GPIO 17
#define LED2_GPIO 19

#define UVC_APP_SCRIPT_NAME "config_pipe.sh"
#define UVC_APP_START_COMMAND                                                  \
    "/home/linaro/workspace/github/aditof_sdk/apps/uvc-app/config_pipe.sh"

static const char *program_name = NULL;
static struct gpio gpio1;
static struct gpio gpio2;
static const int count = 2;

static struct gpio led1;
static struct gpio led2;

static void set_handler(int signal, void (*handler)(int)) {
    struct sigaction sig;
    sigaction(signal, NULL, &sig);
    sig.sa_handler = handler;
    sigaction(signal, &sig, NULL);
}

static void sig_handler(int sig) {
   syslog(LOG_INFO, "%s terminated with signal: %i", program_name, sig);
   closelog();

    gpio_destroy(&gpio1);
    gpio_destroy(&gpio2);
    gpio_destroy(&led1);
    gpio_destroy(&led2);

    exit(EXIT_SUCCESS);
}

void init_daemon(void) {

    pid_t sid;

    set_handler(SIGHUP, sig_handler);
    set_handler(SIGPIPE, sig_handler);
    set_handler(SIGINT, sig_handler);
    set_handler(SIGTERM, sig_handler);

    umask(0);

    openlog("tof-daemon", LOG_NDELAY | LOG_PID, LOG_DAEMON);

    sid = setsid();
    if (sid < 0) {
        syslog(LOG_ERR, "Failed to set session id");
        exit(EXIT_FAILURE);
    }

    const char *working_dir = "/";
    if ((chdir(working_dir)) < 0) {
        syslog(LOG_ERR, "Failed to change current directory to %s",
               working_dir);
        exit(EXIT_FAILURE);
    }

    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
}

void init_buttons(void) {

    gpio_init(&gpio1, BUTTON1_GPIO, true);
    gpio_init(&gpio2, BUTTON2_GPIO, true);
    gpio_init(&led1, LED1_GPIO, false);
    gpio_init(&led2, LED2_GPIO, false);

    gpio_set_edge(&gpio1, "rising");
    gpio_set_edge(&gpio2, "rising");
}

pid_t start_uvc_app() {

    pid_t pid = fork();

    if (pid < 0) {
        exit(EXIT_FAILURE);
    }

    if (pid == 0) {
        execl(UVC_APP_START_COMMAND, "", (char *)NULL);
        exit(1); // In case execl fails
    }

    return pid;
}

void kill_uvc_app() { system("killall -15 uvc-gadget"); }

int main(int argc, char *argv[]) {
    pid_t pid;

    program_name = argv[0];

    pid = fork();
    if (pid < 0) {
        exit(EXIT_FAILURE);
    }

    if (pid > 0) {
        exit(EXIT_SUCCESS);
    }

    init_daemon();
    init_buttons();

    syslog(LOG_INFO, "%s started", program_name);

    struct gpio *gpio_list[count];
    gpio_list[0] = &gpio1;
    gpio_list[1] = &gpio2;

    struct pollfd fds[count];
    int i;
    for (i = 0; i < count; ++i) {
        fds[i].events = POLLPRI | POLLERR;
    }

    bool uvc_app_started = false;
    pid_t uvc_pid;

    while (1) {
        for (i = 0; i < count; ++i) {
            fds[i].fd = open(gpio_list[i]->value_path, O_RDONLY, 0);

            char buf[256];
            lseek(fds[i].fd, 0, SEEK_SET);
            read(fds[i].fd, buf, sizeof(buf));
        }

        int ret = poll(fds, count, -1);

        for (i = 0; i < count; i++) {
            if (fds[i].revents & POLLPRI) {
                char buf[256];
                lseek(fds[i].fd, 0, SEEK_SET);
                read(fds[i].fd, buf, sizeof(buf));

                int ch = 0;
                while (buf[ch] != '\n' && ch < sizeof(buf)) {
                    ch++;
                }
                if (ch < sizeof(buf))
                    buf[ch] = '\0';
                else
                    buf[255] = '\0';
                gpio_list[i]->value = atoi(buf);
            } else {
                gpio_list[i]->value = -1;
            }
        }

        for (i = 0; i < count; ++i)
            close(fds[i].fd);

        for (i = 0; i < count; i++) {
            if (gpio_list[i]->value != -1) {
                switch (gpio_list[i]->id) {
                case BUTTON1_GPIO: {
                    // Button 1 pressed. Do stuff.
                    break;
                }
                case BUTTON2_GPIO: {
                    if (uvc_app_started) {
                        kill(uvc_pid, 15);
                        waitpid(uvc_pid, NULL, 0);
                        kill_uvc_app();
                    } else {
                        uvc_pid = start_uvc_app();
                    }

                    uvc_app_started = !uvc_app_started;
                    gpio_set_value(&led2, uvc_app_started ? 1 : 0);
                    break;
                }
                }
            }
        }
    }

    exit(EXIT_SUCCESS);
};
