/*
 * swm_menu.c
 *
 *  Created on: Mar 4, 2020
 *      Author: kurt
 */

#include "swm_menu.h"
#include "swm_common.h"

int get_run_flag(char *pstr)
{
    if( *pstr != '-' ) return -1;

    if(strncmp(pstr+1, "save", 4)==0)  {
        return RUN_FLAG_SAVE;
    }
    else if(strncmp(pstr+1, "debug", 5)==0)  {
        return RUN_FLAG_DEBUG;
    }
    else if (strncmp(pstr+1, "wpath", 5)==0) {
        return RUN_FLAG_WPATH;
    }

    return RUN_FLAG_NONE;
}
