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

#include "swm_define.h"
#include "swm_ldm_data.h"

static pthread_mutex_t  mutex_ldm_monitor;

static std::map<string, swm_ldm::st_TRSIGNAL>    traffic_signal_data_list;

int add_traffic_signal_data_list(swm_ldm::st_TRSIGNAL& traffic_light_signal)
{
    auto search = traffic_signal_data_list.find(traffic_light_signal.trafficLightId);

    if (search == traffic_signal_data_list.end())
    {
        traffic_signal_data_list.insert({traffic_light_signal.trafficLightId, traffic_light_signal});
    }
    else
    {
        search->second = traffic_light_signal;
    }
    return 0;
}

std::map<string, swm_ldm::st_TRSIGNAL>& get_traffic_signal_data_list(void)
{
    return traffic_signal_data_list;
}

void clear_traffic_signal_data_list(void)
{
    traffic_signal_data_list.clear();
}

static std::vector<swm_ldm::st_RSA>  rsa_data_list;

int add_rsa_data_list(swm_ldm::st_RSA& rsa)
{
#if 1
    bool is_found = false;
    for (auto& data : rsa_data_list)
    {
        if (data.itis == rsa.itis)
        {
            if (data.latitude == rsa.latitude && data.longitude == rsa.longitude)
            {
                data.extent = rsa.extent;
                is_found = true;
            }
        }
    }

    if (!is_found)
    {
        rsa_data_list.push_back(rsa);
    }
#else
    rsa_data_list.push_back(rsa);
#endif
    return 0;
}

void clear_rsa_data_list(void)
{
    rsa_data_list.clear();
}

std::vector<swm_ldm::st_RSA>& get_rsa_data_list(void)
{
    return rsa_data_list;
}

static std::vector<swm_ldm::st_EVA> eva_data_list;

int add_eva_data_list(swm_ldm::st_EVA& eva)
{
    int count = 0;
    for (auto& data : eva_data_list)
    {
        if (memcmp(&data.id[0], &eva.id[0], sizeof(u8) * 4) == 0)
        {
            break;
        }
        count++;
    }

    if (eva_data_list.size() > count)
        eva_data_list.erase(eva_data_list.begin() + count);

    eva_data_list.push_back(eva);
}

void clear_eva_data_list(void)
{
    eva_data_list.clear();
}

std::vector<swm_ldm::st_EVA>& get_eva_data_list(void)
{
    return eva_data_list;
}

static std::vector<swm_ldm::st_BSM> bsm_data_list;

int add_bsm_data_list(swm_ldm::st_BSM& bsm)
{
    int count = 0;
    for (auto& data : bsm_data_list)
    {
        if (memcmp(&data.id[0], &bsm.id[0], sizeof(u8) * 4) == 0)
        {
            break;
        }
        count++;
    }

    if (bsm_data_list.size() > count)
        bsm_data_list.erase(bsm_data_list.begin() + count);
    
    bsm_data_list.push_back(bsm);
}

void clear_bsm_data_list(void)
{
   bsm_data_list.clear();
}

std::vector<swm_ldm::st_BSM>& get_bsm_data_list(void)
{
    return bsm_data_list;
}

static std::vector<swm_ldm::st_MOBJ> mobj_data_list;

int add_mobj_data_list(swm_ldm::st_MOBJ& mobj)
{
    int count = 0;
    for (auto& data : mobj_data_list)
    {
        if (memcmp(&data.idmoving, &mobj.idmoving, sizeof(u32)) == 0)
        {
            break;
        }
        count++;
    }

    if (mobj_data_list.size() > count)
        mobj_data_list.erase(mobj_data_list.begin() + count);
    
    mobj_data_list.push_back(mobj);
}

void clear_mobj_data_list(void)
{
   mobj_data_list.clear();
}

std::vector<swm_ldm::st_MOBJ>& get_mobj_data_list(void)
{
    return mobj_data_list;
}

pthread_mutex_t* get_mutex_ldm_monitor(void)
{
    return &mutex_ldm_monitor;
}

void ldm_monitor_lock(void)
{
    pthread_mutex_lock(&mutex_ldm_monitor);
}

void ldm_monitor_unlock(void)
{
    pthread_mutex_unlock(&mutex_ldm_monitor);
}

void getcurtime(struct timeval *ot)
{
	gettimeofday(ot, NULL);
}

long gettimediff(struct timeval *ot, struct timeval *dt)
{
	long diff_ms=0;
	struct timeval ct;

	getcurtime(&ct);
	dt->tv_sec =  ct.tv_sec - ot->tv_sec;
	if(ct.tv_usec>=ot->tv_usec)
		dt->tv_usec  =  ct.tv_usec  - ot->tv_usec;
	else {
		dt->tv_sec--;
		dt->tv_usec = (1000000 - ot->tv_usec)+ct.tv_usec;
	}
	diff_ms = dt->tv_sec * 1000 + (dt->tv_usec/1000);

	return diff_ms;
}