//////////////////////////////////////////////////////////////////

//
// Created  : 7/7/2022, b. t. tseng 
// Edit     : 7/26/2022, b. t. tseng
//
//////////////////////////////////////////////////////////////////

#pragma once


#include <SDKDDKVer.h>

#include <stdio.h>
#include <windows.h>
#include <tchar.h>
#include <rtapi.h>    // RTX64 APIs that can be used in real-time and Windows applications.

#ifdef UNDER_RTSS
#include <rtssapi.h>  // RTX64 APIs that can only be used in real-time applications.
#endif // UNDER_RTSS
//#include"ksm64shared.h"
#include "ksapi.h"
#include "ksmotion.h"
#define RTSS
#include "shared.hpp"
//#include"../libs/shared.h"
#include "RT_Robot_Controller.h"
#include "rt_cyclic_routine.h"
//#include"rt_robot_controller.h"
//#include"rt_cyclic_routine.h"
//
 
struct AppDirectory {
    bool a;
    char app_directory[128];
    DWORD len;
    
};


