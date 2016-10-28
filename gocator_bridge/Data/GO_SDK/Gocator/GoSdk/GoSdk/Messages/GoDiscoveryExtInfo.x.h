/** 
 * @file    GoDiscoveryExtInfo.x.h
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DISCOVERY_EXT_INFO_X_H
#define GO_SDK_DISCOVERY_EXT_INFO_X_H

#include <kApi/Io/kSerializer.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kBytes.h>
#include <GoSdk/Internal/GoDiscovery.h>

kBeginHeader()

kDeclareValue(Go, GoDiscoveryProperty, kValue)

typedef struct GoDiscoveryExtInfoClass
{
    kObjectClass base;
    
    k32u id;
    GoAddressInfo address;
    GoPortInfo ports;
    kVersion version;
    k64u uptime;
    kArrayList properties;
} GoDiscoveryExtInfoClass;

kDeclareClass(Go, GoDiscoveryExtInfo, kObject)

GoFx(kStatus) GoDiscoveryExtInfo_Construct(GoDiscoveryExtInfo* msg, kAlloc allocator);
GoFx(kStatus) GoDiscoveryExtInfo_Init(GoDiscoveryExtInfo msg, kType type, kAlloc alloc);
GoFx(kStatus) GoDiscoveryExtInfo_VInitClone(GoDiscoveryExtInfo msg, GoDiscoveryExtInfo source, kAlloc alloc); 
GoFx(kStatus) GoDiscoveryExtInfo_AllocateProperties(GoDiscoveryExtInfo msg, kSize count);
GoFx(kStatus) GoDiscoveryExtInfo_VRelease(GoDiscoveryExtInfo msg);
GoFx(kSize) GoDiscoveryExtInfo_VSize(GoDiscoveryExtInfo msg); 
GoFx(kStatus) GoDiscoveryExtInfo_Read(GoDiscoveryExtInfo msg, kSerializer serializer, kAlloc alloc);

#define GoDiscoveryExtInfo_(D)                          kCast(GoDiscoveryExtInfoClass*, D)
#define GoDiscoveryExtInfo_SetContent_(D, V)            (GoDiscoveryExtInfo_(D) = (V), kOK)
#define GoDiscoveryExtInfo_Content_(D)                  (GoDiscoveryExtInfo_(D))
#define GoDiscoveryExtInfo_SetSource_(D, V)             (GoDiscoveryExtInfo_(D)->source = (V), kOK)

kEndHeader()

#endif