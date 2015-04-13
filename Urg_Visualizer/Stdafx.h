// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently

#pragma once

#include <math.h>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Urg_driver.h>
#include <Connection_information.h>

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <cstdio>

#define MOCKABLE virtual

namespace boost { 
    namespace detail { 
        namespace win32 { 
            struct _SECURITY_ATTRIBUTES: public ::_SECURITY_ATTRIBUTES {}; 
        };
    };
};


