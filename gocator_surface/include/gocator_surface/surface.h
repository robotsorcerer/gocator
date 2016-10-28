#pragma once
#ifndef __SURFACE_H__
#define __SURFACE_H__

#include <kApi/kApi.h>

GoPartDetection detection = kNULL;
k64f height = 300;   //in mm
kBool enable = true; //enable edge filtering
k64f EdgeFilterWidth = 5.0 ;//5mm
k64f EdgeFilterLength = 5.0; //5mm
k64f minArea = 5;//5mm^2
k64f maxArea = 500; //200mm^2

GoSurfaceGeneration surface;
//When Type is set to Continuous, part detection is automatically enabled.
GoSurfaceGenerationType surfaceType =  GO_SURFACE_GENERATION_TYPE_CONTINUOUS  ;

#endif