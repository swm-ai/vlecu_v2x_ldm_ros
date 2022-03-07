#include "ros/ros.h"
#include "swm_ldm/st_SWMLDM.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <time.h>
#include <ncurses.h>

#include "swm_define.h"
#include "swm_traffic_light.h"
#include "swm_monitor_window_proc.h"

const char* get_traffic_signal(u8 state)
{
    switch (state)
    {
    case TRAFFIC_LIGHT_COLOR_RED:
        return "Red";
        break;
    case TRAFFIC_LIGHT_COLOR_YELLOW:
        return "Yellow";
        break;
    case TRAFFIC_LIGHT_COLOR_GREEN:
        return "Green";
        break;
    case TRAFFIC_LIGHT_COLOR_RED_LEFT:
        return "Left";
        break;
    case TRAFFIC_LIGHT_COLOR_GREEN_LEFT:
        return "Green and Left";
        break;
    case TRAFFIC_LIGHT_COLOR_BLACK:
    case TRAFFIC_LIGHT_COLOR_UNKNOWN:
    default:
        break;
    }
    return "unknown";
}

void output_traffic_signal_string(int ev_state)
{
    switch (ev_state)
    {
    case TRAFFIC_LIGHT_COLOR_RED:
        cprintf(COLOR_RED, "  RED        ");
        break;
    case TRAFFIC_LIGHT_COLOR_YELLOW:
        cprintf(COLOR_YELLOW, "  YELLOW     ");
        break;
    case TRAFFIC_LIGHT_COLOR_GREEN:
        cprintf(COLOR_GREEN, "  GREEN      ");
        break;
    case TRAFFIC_LIGHT_COLOR_RED_LEFT:
        cprintf(COLOR_WHITE, "  RED_LEFT   ");
        break;
    case TRAFFIC_LIGHT_COLOR_GREEN_LEFT:
        cprintf(COLOR_BLUE, "  GREEN_LEFT ");
        break;
    case TRAFFIC_LIGHT_COLOR_BLACK:
    case TRAFFIC_LIGHT_COLOR_UNKNOWN:
    default:
        printw("  UNKNOWN    ");
        break;
    }
}