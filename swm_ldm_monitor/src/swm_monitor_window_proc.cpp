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
#include <unistd.h>

#include "swm_define.h"
#include "swm_ldm_data.h"
#include "swm_traffic_light.h"
#include "swm_road_side_alert.h"
#include "swm_emergency_vehicle_alert.h"
#include "swm_monitor_window_proc.h"

static bool is_alive = true;

void *ldm_monitor_window_proc(void* args)
{
    static int retval = 0xf1;
    pthread_mutex_t* ldm_mutex = get_mutex_ldm_monitor();
    //struct timeval clr_tv = {0,}, df_tv = {0,};

    pthread_mutex_init(ldm_mutex, NULL);
    swm_color_init();

    //getcurtime(&clr_tv);

    while (is_alive)
    {
#if 0
        // after 5 seconds, clear windows
        usleep(5000000);
        clear();
#else
        clear();
        //test_traffic_signal_process();    // this code is for test.
        //test_road_side_alert();   // this code is for test.
        
        display_ldm_monitor_title();
        printw("\r\n\r\n");
        display_traffic_light_signal();
        printw("\r\n\r\n");
        display_road_side_alert();
        printw("\r\n\r\n");
        display_emergency_vehicle_alert();
        printw("\r\n\r\n");
        display_bsm_data();
        printw("\r\n\r\n");
        display_mobj_data();
        refresh();
        usleep(1000 * 1000);
        /*usleep(50 * 1000);
        if (gettimediff(&clr_tv, &df_tv) > 1500)
        {
            getcurtime(&clr_tv);
            ldm_monitor_lock();
            clear_traffic_signal_data_list();
            clear_rsa_data_list();
            ldm_monitor_unlock();
        }*/
#endif
    }
    //test();

    endwin();

    pthread_exit(&retval);
}

void ldm_monitor_window_close(void)
{
    is_alive = false;
}

bool ldm_monitor_windows_alive(void)
{
    return is_alive;
}

void ldm_monitor_windows_display(void)
{
    clear();
    display_ldm_monitor_title();
    printw("\r\n\r\n");
    display_traffic_light_signal();
    printw("\r\n\r\n");
    display_road_side_alert();
    refresh();
}

void display_ldm_monitor_title(void)
{
    printw("+--------------------------+\n");
    printw("| LDM INFORMATION MONNITOR |\n");
    printw("+--------------------------+\n");
}

void display_traffic_light_signal(void)
{
    ldm_monitor_lock();
    std::map<string, swm_ldm::st_TRSIGNAL>& data_list = get_traffic_signal_data_list();
    if (data_list.size() <= 0) {
        ldm_monitor_unlock();
        return;
    }
    printw("+----------------------------------------------+\n");
    printw("| Traffic light signal                         |\n");
    printw("+------------------+-------------+-------------+\n");
    printw("| Traffic light id | light color | remain time |\n");
    printw("+------------------+-------------+-------------+\n");
    for (auto& element : data_list)
    {
        swm_ldm::st_TRSIGNAL traffic_data = element.second;
        printw("|   %s   |", traffic_data.trafficLightId.c_str());
        output_traffic_signal_string(traffic_data.evState);
        printw("|%4d seconds |\n", traffic_data.endTime);
        printw("+------------------+-------------+-------------+\n");
    }
    clear_traffic_signal_data_list();
    ldm_monitor_unlock();
}

void display_road_side_alert(void)
{
    ldm_monitor_lock();
    std::vector<swm_ldm::st_RSA>& data_list = get_rsa_data_list();
    if (data_list.size() <= 0)
    {
        ldm_monitor_unlock();
        return;
    }
    printw("+------------------------------------------------------------------------------------------------+\n");
    printw("| Road side alert                                                                                |\n");
    printw("+--------------------------------------+-----------------+---------------------------------------+\n");
    printw("|              ITIS CODE               |    latitude     |    longitude    |        extent       |\n");
    printw("+--------------------------------------+-----------------+-----------------+---------------------+\n");
    for (auto& element : data_list)
    {
        swm_ldm::st_RSA rsa_data = element;
        printw("|");
        cprintf(COLOR_RED, " %36s ", get_road_side_alert(rsa_data.itis));
        //output_road_side_alert_string();
        printw("|%16.7llf |", rsa_data.latitude / 10000000.0L);
        printw("%16.7llf |", rsa_data.longitude / 10000000.0L);
        printw("  %18s |\n", get_extent_string(rsa_data.extent));
        printw("+--------------------------------------+-----------------+-----------------+---------------------+\n");
    }
    clear_rsa_data_list();
    ldm_monitor_unlock();
}

void display_emergency_vehicle_alert(void)
{
    ldm_monitor_lock();
    std::vector<swm_ldm::st_EVA>& data_list = get_eva_data_list();
    if (data_list.size() <= 0)
    {
        ldm_monitor_unlock();
        return;
    }
    printw("+--------------------------------------------------------------------------------------+\n");
    printw("| Emergency vehicle alert                                                              |\n");
    printw("+----------+--------------------------+-----------+---------------------+--------------+\n");
    printw("|vehicle id|       response type      |   siren   |      light bar      | multi vehicle|\n");
    printw("+----------+--------------------------+-----------+---------------------+--------------+\n");
    for (auto& element : data_list)
    {
        swm_ldm::st_EVA eva_data = element;
        printw("|");
        printw("0x%02X%02X%02X%02X|", eva_data.id[0], eva_data.id[1], eva_data.id[2], eva_data.id[3]); //10
        cprintf(COLOR_RED, "%26s", get_response_type(eva_data.response_type)); // max 26
        printw("|%11s|", get_sirent_state(eva_data.details[0].siren_use)); // max 11
        printw("%21s|", get_light_bar_state(eva_data.details[0].lights_use)); // max 21
        printw("%14s|\n", get_multiple_vehicle_type(eva_data.details[0].multi)); // max 14
        printw("+----------+--------------------------+-----------+---------------------+--------------+\n");
    }
    clear_eva_data_list();
    ldm_monitor_unlock();
}

void display_bsm_data(void)
{
    double latitude = 0.0, longitude = 0.0;
    float heading = 0;
    int speed = 0;

    ldm_monitor_lock();
    std::vector<swm_ldm::st_BSM>& data_list = get_bsm_data_list();
    if (data_list.size() <= 0)
    {
        ldm_monitor_unlock();
        return;
    }
    printw("+----------------------------------------------------------------+\n");
    printw("| Near vehicle information (BSM)                                 |\n");
    printw("+------------+--------------+--------------+---------+-----------+\n");
    printw("| vehicle id |   latitude   |   longitude  | heading | speed(km) |\n");
    printw("+------------+--------------+--------------+---------+-----------+\n");
    for (auto& element : data_list)
    {
        swm_ldm::st_BSM bsm_data = element;
        latitude = (double)bsm_data.lat;
        longitude = (double)bsm_data.longi;
        heading = ((float)bsm_data.heading) / 80;
        speed = (bsm_data.speed * 9) / 125;
        printw("| ");
        cprintf(COLOR_RED, "0x%02X%02X%02X%02X", bsm_data.id[0], bsm_data.id[1], bsm_data.id[2], bsm_data.id[3]); //10
        printw(" | %12.7lf | %12.7lf | %7.3f |    %03d    |\n", latitude, longitude, heading, speed); // (12 + 12 + 6 + 3 = 36) + 5
        printw("+------------+--------------+--------------+---------+-----------+\n");
    }
    clear_bsm_data_list();
    ldm_monitor_unlock();
}

void display_mobj_data(void)
{
    int mobjid = 0;
    double latitude = 0.0, longitude = 0.0;

    ldm_monitor_lock();
    std::vector<swm_ldm::st_MOBJ>& data_list = get_mobj_data_list();
    if (data_list.size() <= 0);
    {
        ldm_monitor_unlock();
        return;
    }
    printw("+------------------------------------------+\n");
    printw("| Moving object information (MOBJ)         |\n");
    printw("+------------+--------------+--------------+\n");
    printw("| mobject id |   latitude   |   longitude  |\n");
    printw("+------------+--------------+--------------+\n");
    for (auto& element : data_list)
    {
        swm_ldm::st_MOBJ mobj_data = element;
        mobjid = mobj_data.idmoving;
        latitude = (double)mobj_data.latitude;
        longitude = (double)mobj_data.longitude;
        printw("| ");
        cprintf(COLOR_RED, "0x%08X", mobjid);
        printw(" | %12.7lf | %12.7lf |\n", latitude, longitude);
        printw("+------------+--------------+--------------+\n");
    }
    clear_mobj_data_list();
    ldm_monitor_unlock();
}

void test_traffic_signal_process(void)
{
    static u32 count = 0;
    int temp_count = 0;

    count++;

    ldm_monitor_lock();
    for (int i = 0; i < 4; i ++)
    {
        swm_ldm::st_TRSIGNAL temp_signal = swm_ldm::st_TRSIGNAL();
        switch (i)
        {
        case 0:
            {
                temp_signal.trafficLightId = std::string("C119AW000021");
                temp_signal.evState = (count % 40) > 20 ? TRAFFIC_LIGHT_COLOR_GREEN : TRAFFIC_LIGHT_COLOR_RED;
                temp_signal.endTime = count % 20;
                break;
            }
        case 1:
            {
                temp_signal.trafficLightId = std::string("C119AW000022");
                temp_signal.evState = (count % 50) > 25 ? TRAFFIC_LIGHT_COLOR_YELLOW : TRAFFIC_LIGHT_COLOR_RED_LEFT;
                temp_signal.endTime = count % 25;
                break;
            }
        case 2:
            {
                temp_signal.trafficLightId = std::string("C119AW000024");
                temp_signal.evState = (temp_count = count % 120) > 90 ? TRAFFIC_LIGHT_COLOR_GREEN_LEFT : 
                                            (temp_count > 60 ? TRAFFIC_LIGHT_COLOR_RED_LEFT :
                                                (temp_count > 30 ? TRAFFIC_LIGHT_COLOR_YELLOW :
                                                    TRAFFIC_LIGHT_COLOR_RED));
                temp_signal.endTime = count % 30;
                break;
            }
        case 3:
            {
                temp_signal.trafficLightId = std::string("C119AW000023");
                temp_signal.evState = (count % 100) > 50 ? TRAFFIC_LIGHT_COLOR_RED : TRAFFIC_LIGHT_COLOR_GREEN;
                temp_signal.endTime = count % 50;
                break;
            }
        default:
            break;
        }
        add_traffic_signal_data_list(temp_signal);
    }
    ldm_monitor_unlock();
}

void test_road_side_alert(void)
{
    static u32 count = 0;
    int check_time = 0;
    count++;

    ldm_monitor_lock();
    check_time = count % 120;
    swm_ldm::st_RSA rsa_data;
    if (check_time < 20)
    {
        rsa_data = swm_ldm::st_RSA();
        rsa_data.itis = ITIS_CODE_OBSTRUCTION_ON_ROADWAY;
        rsa_data.latitude = 372427654;
        rsa_data.longitude = 1267737169;
        add_rsa_data_list(rsa_data);
    }
    else if (check_time < 40)
    {
        rsa_data = swm_ldm::st_RSA();
        rsa_data.itis = ITIS_CODE_PARKING;
        rsa_data.latitude = 372427654;
        rsa_data.longitude = 1267737169;
        add_rsa_data_list(rsa_data);
    }
    else if (check_time < 60)
    {
        rsa_data = swm_ldm::st_RSA();
        rsa_data.itis = ITIS_CODE_PEOPLE_ON_ROADWAY;
        rsa_data.latitude = 372427654;
        rsa_data.longitude = 1267737169;
        add_rsa_data_list(rsa_data);
    }
    ldm_monitor_unlock();
}

int test(){
    int count = 0;

    while(is_alive){
        clear();
        cprintf(COLOR_GREEN, "#############################################\n");
        cprintf((count % 6) + 1, "\n\n\n\n\n\n##### %d #####", ++count);
        cprintf(COLOR_CYAN, " HaHa");
        cprintf(COLOR_YELLOW, "HoHo");
        cprintf(COLOR_MAGENTA, "KaKa \n");
        cprintf(COLOR_RED, "\n\n\n\n\n\n#############################################\n");
        printw("Haha\n");
        refresh();
        usleep(1000 * 1000);
    }

    return 0;
}

void swm_color_init(void)
{
    initscr();
    noecho();
    curs_set(FALSE);

    if (has_colors())
    {
        start_color();
        init_pair(COLOR_RED, COLOR_BLACK, COLOR_RED);
        init_pair(COLOR_GREEN, COLOR_BLACK, COLOR_GREEN);
        init_pair(COLOR_YELLOW, COLOR_BLACK, COLOR_YELLOW);
        init_pair(COLOR_BLUE, COLOR_BLACK, COLOR_BLUE);
        init_pair(COLOR_MAGENTA, COLOR_BLACK, COLOR_MAGENTA);
        init_pair(COLOR_CYAN, COLOR_BLACK, COLOR_CYAN);
        init_pair(COLOR_WHITE, COLOR_BLACK, COLOR_WHITE);
    }
    start_color();
}

void cprintf(int color, const char* fmt, ...)
{
    int nlen = 0;
    char buf[1024];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    
    nlen = strlen(buf);

    for (int i = 0; i < nlen; i++) addch(buf[i] | COLOR_PAIR(color));
    //refresh();
}
