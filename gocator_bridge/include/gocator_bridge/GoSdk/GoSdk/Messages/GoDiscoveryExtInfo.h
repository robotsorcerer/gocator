/** 
 * @file    GoDiscoveryExtInfo.h
 * @brief   Declares the GoDiscoveryExtInfo class and related types.
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DISCOVERY_EXT_INFO_H
#define GO_SDK_DISCOVERY_EXT_INFO_H

#include <GoSdk/GoSdkDef.h>
kBeginHeader()

/**
 * @class   GoDiscoveryExtInfo
 * @extends kObject
 * @ingroup GoSdk-Discovery
 * @brief   Represents an extended Discovery Information object.
 */
typedef kObject GoDiscoveryExtInfo; 

/**
 * @struct  GoDiscoveryProperty
 * @extends kValue
 * @ingroup GoSdk-Discovery
 * @brief   Represents a property returned in an extended Discovery information message.
 */
typedef struct GoDiscoveryProperty
{
    kText256 name;
    kText256 value;
} GoDiscoveryProperty;

/** 
 * Gets the source device ID.
 *
 * @public             @memberof GoDiscoveryExtInfo
 * @version            Introduced in firmware 4.3.3.124
 * @param   msg        Message object. 
 * @return             Source device ID. 
 */
GoFx(k32u) GoDiscoveryExtInfo_Id(GoDiscoveryExtInfo msg);

/** 
 * Gets the address information returned by the source device.
 *
 * @public             @memberof GoDiscoveryExtInfo
 * @version            Introduced in firmware 4.3.3.124
 * @param   msg        Message object. 
 * @return             Address information. 
 */
GoFx(GoAddressInfo) GoDiscoveryExtInfo_Address(GoDiscoveryExtInfo msg);

/** 
 * Gets a set of ports used by the source device.
 *
 * @public             @memberof GoDiscoveryExtInfo
 * @version            Introduced in firmware 4.3.3.124
 * @param   msg        Message object. 
 * @return             Indicator pointer.
 * @remark             If a port has a value of k16U_NULL, then the associated port is not used.
 */
GoFx(GoPortInfo) GoDiscoveryExtInfo_Ports(GoDiscoveryExtInfo msg);

/** 
 * Retrieves the application version of the source device.
 *
 * @public             @memberof GoDiscoveryExtInfo
 * @version            Introduced in firmware 4.3.3.124
 * @param   msg        Message object. 
 * @return             Application version.
 */
GoFx(kVersion) GoDiscoveryExtInfo_Version(GoDiscoveryExtInfo msg);

/** 
 * Retrieves the up time of the source device.
 *
 * @public             @memberof GoDiscoveryExtInfo
 * @version            Introduced in firmware 4.3.3.124
 * @param   msg        Message object. 
 * @return             Device up time in microseconds.
 */
GoFx(k64u) GoDiscoveryExtInfo_UpTime(GoDiscoveryExtInfo msg);

/** 
 * Retrieves the number of properties present in the extended discovery information message.
 *
 * @public             @memberof GoDiscoveryExtInfo
 * @version            Introduced in firmware 4.3.3.124
 * @param   msg        Message object. 
 * @return             The property count of the extended discovery information message.
 */
GoFx(kSize) GoDiscoveryExtInfo_PropertyCount(GoDiscoveryExtInfo msg);

/** 
 * Retrieves the property associated with the given index or kNULL if invalid.
 *
 * @public             @memberof GoDiscoveryExtInfo
 * @version            Introduced in firmware 4.3.3.124
 * @param   msg        Message object. 
 * @param   index      The index of the property to retrieve.
 * @return             The property at the given index or kNULL.
 */
GoFx(const GoDiscoveryProperty*) GoDiscoveryExtInfo_PropertyAt(GoDiscoveryExtInfo msg, kSize index);

kEndHeader()
#include <GoSdk/Messages/GoDiscoveryExtInfo.x.h>

#endif
