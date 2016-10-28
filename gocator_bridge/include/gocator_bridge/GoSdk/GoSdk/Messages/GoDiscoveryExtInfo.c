/** 
 * @file    GoDiscoveryExtInfo.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Messages/GoDiscoveryExtInfo.h>


kBeginValue(Go, GoDiscoveryProperty, kValue)
    kAddField(GoDiscoveryProperty, kText256, name)
    kAddField(GoDiscoveryProperty, kText256, value)
kEndValue()


/* 
 * GoDiscoveryExtInfo
 */

kBeginClass(Go, GoDiscoveryExtInfo, kObject) 
    //virtual methods
    kAddVMethod(GoDiscoveryExtInfo, kObject, VInitClone)
    kAddVMethod(GoDiscoveryExtInfo, kObject, VRelease)
    kAddVMethod(GoDiscoveryExtInfo, kObject, VSize)

kEndClass() 

GoFx(kStatus) GoDiscoveryExtInfo_Construct(GoDiscoveryExtInfo* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback_(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoDiscoveryExtInfo), msg)); 

    if (!kSuccess(status = GoDiscoveryExtInfo_Init(*msg, kTypeOf(GoDiscoveryExtInfo), alloc)))
    {
        kAlloc_FreeRef(alloc, msg); 
    }

    return status; 
}

GoFx(kStatus) GoDiscoveryExtInfo_Init(GoDiscoveryExtInfo msg, kType type, kAlloc alloc)
{
    GoDiscoveryExtInfoClass* obj = msg; 

    kCheck(kObject_Init_(msg, type, alloc)); 
    kInitFields_(GoDiscoveryExtInfo, msg);
        
    return kOK; 
}

GoFx(kStatus) GoDiscoveryExtInfo_VInitClone(GoDiscoveryExtInfo msg, GoDiscoveryExtInfo source, kAlloc alloc)
{
    GoDiscoveryExtInfoClass* obj = msg; 
    GoDiscoveryExtInfoClass* srcObj = source; 
    kStatus status; 

    kCheck(GoDiscoveryExtInfo_Init(msg, kObject_Type_(source), alloc));     

    kTry
    {
        obj->id = srcObj->id;
        obj->version = srcObj->version;
        obj->uptime = srcObj->uptime;
        kMemCopy(&(obj->address), &(srcObj->address), sizeof(GoAddressInfo));
        kMemCopy(&(obj->ports), &(srcObj->ports), sizeof(GoPortInfo));
        
        kTest(kObject_Clone(&obj->properties, srcObj->properties, alloc)); 
    }
    kCatch(&status)
    {
        GoDiscoveryExtInfo_VRelease(msg); 
        kEndCatch(status); 
    }

    return kOK; 
}

GoFx(kStatus) GoDiscoveryExtInfo_AllocateProperties(GoDiscoveryExtInfo msg, kSize count)
{
    GoDiscoveryExtInfoClass* obj = msg; 

    kCheck(kDisposeRef(&obj->properties)); 
    kCheck(kArrayList_Construct(&obj->properties, kTypeOf(GoDiscoveryProperty), count, kObject_Alloc_(msg))); 

    return kOK; 
}

GoFx(kStatus) GoDiscoveryExtInfo_VRelease(GoDiscoveryExtInfo msg)
{
    GoDiscoveryExtInfoClass* obj = msg; 

    kCheck(kDisposeRef(&obj->properties));

    return kObject_VRelease(msg); 
}

GoFx(kSize) GoDiscoveryExtInfo_VSize(GoDiscoveryExtInfo msg)
{
    GoDiscoveryExtInfoClass* obj = msg; 

    return sizeof(GoDiscoveryExtInfoClass) + (kIsNull(obj->properties) ? 0 : kObject_Size_(obj->properties)); 
}

GoFx(k32u) GoDiscoveryExtInfo_SubnetPrefixToMask(k32u prefix)
{
    k32u mask = 0;
    k32s lower = 32 - (k32s)prefix;
    k32s i = 0;

    for (i = 31; i >= lower; --i)
    {
        mask |= (1 << i);
    }

    return mask;
}


GoFx(kStatus) GoDiscoveryExtInfo_Read(GoDiscoveryExtInfo msg, kSerializer serializer, kAlloc alloc)
{
    GoDiscoveryExtInfoClass* obj = msg;
    k32u size, responseId, temp, prefixLength;
    k64u signature;
    kByte useDhcp, tempByte;
    k32s status;
    k16u attrSize;
    kByte tempArray[12];
    k8u propertyCount;

    kCheck(GoDiscoveryExtInfo_Init(msg, kTypeOf(GoDiscoveryExtInfo), alloc));

    //construct extended discovery info instance
    kCheck(kSerializer_Read32u(serializer, &size));
    kCheck(kSerializer_Read32u(serializer, &temp));

    kCheck(kSerializer_Read32u(serializer, &responseId));
    if (responseId != GO_DISCOVERY_GET_INFO_REPLY)
    {
        kCheck(kSerializer_AdvanceRead(serializer, size - (sizeof(k32u) * 3)));
        return kERROR_COMMAND;
    }

    kCheck(kSerializer_Read32u(serializer, &temp));

    kCheck(kSerializer_Read32s(serializer, &status));
    kCheck(status);
    kCheck(kSerializer_Read32u(serializer, &temp));

    kCheck(kSerializer_Read64u(serializer, &signature));
    kCheck(signature == GO_DISOVERY_SIGNATURE);

    kCheck(kSerializer_Read16u(serializer, &attrSize));
    kCheck(attrSize == GO_DISCOVERY_GET_INFO_REPLY_ATTR_SIZE);

    kCheck(kSerializer_Read32u(serializer, &obj->id));
    kCheck(kSerializer_Read32u(serializer, &obj->version));
    kCheck(kSerializer_Read64u(serializer, &obj->uptime));
    kCheck(kSerializer_ReadByte(serializer, &useDhcp));
    obj->address.useDhcp = (kBool)useDhcp;

    kCheck(kSerializer_ReadByte(serializer, &tempByte));
    obj->address.address.version = (kIpVersion)tempByte;
    kCheck(kSerializer_ReadByteArray(serializer, obj->address.address.address, 4));
    kCheck(kSerializer_ReadByteArray(serializer, tempArray, 12));

    kCheck(kSerializer_Read32u(serializer, &prefixLength));
    obj->address.mask = kIpAddress_FromHost32u(GoDiscoveryExtInfo_SubnetPrefixToMask(prefixLength));    

    kCheck(kSerializer_ReadByte(serializer, &tempByte));
    obj->address.gateway.version = (kIpVersion)tempByte;
    kCheck(kSerializer_ReadByteArray(serializer, obj->address.gateway.address, 4));
    kCheck(kSerializer_ReadByteArray(serializer, tempArray, 12));

    kCheck(kSerializer_Read16u(serializer, &obj->ports.controlPort));
    kCheck(kSerializer_Read16u(serializer, &obj->ports.upgradePort));
    kCheck(kSerializer_Read16u(serializer, &obj->ports.healthPort));
    kCheck(kSerializer_Read16u(serializer, &obj->ports.dataPort));
    kCheck(kSerializer_Read16u(serializer, &obj->ports.webPort));

    kCheck(kSerializer_Read8u(serializer, &propertyCount));

    if (propertyCount > 0)
    {
        kSize i;
        kStatus exception = kOK;

        kCheck(GoDiscoveryExtInfo_AllocateProperties(msg, propertyCount));
        
        for (i = 0; i < propertyCount; i++)
        {
            k8u nameLength, valueLength;
            GoDiscoveryProperty* property = (GoDiscoveryProperty*)kArrayList_At(obj->properties, i);//kNULL;

            kTry
            {
                kTest(kSerializer_Read8u(serializer, &nameLength));
                kTest(kSerializer_ReadCharArray(serializer, property->name, nameLength));

                kTest(kSerializer_Read8u(serializer, &valueLength));
                kTest(kSerializer_ReadCharArray(serializer, property->value, valueLength));
            }
            kCatch(&exception)
            {
                kMemFree(property);
                kEndCatch(exception);
            }
        }

        kCheck(kArrayList_AddCount(obj->properties, propertyCount));
    }

    return kOK; 
}

GoFx(k32u) GoDiscoveryExtInfo_Id(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->id;
}

GoFx(GoAddressInfo) GoDiscoveryExtInfo_Address(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->address; 
}

GoFx(GoPortInfo) GoDiscoveryExtInfo_Ports(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->ports;
}

GoFx(kVersion) GoDiscoveryExtInfo_Version(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->version;
}

GoFx(k64u) GoDiscoveryExtInfo_UpTime(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->uptime;
}

GoFx(kSize) GoDiscoveryExtInfo_PropertyCount(GoDiscoveryExtInfo msg)
{
    return kArrayList_Count(GoDiscoveryExtInfo_Content_(msg)->properties);
}

GoFx(const GoDiscoveryProperty*) GoDiscoveryExtInfo_PropertyAt(GoDiscoveryExtInfo msg, kSize index)
{
    if (index >= kArrayList_Count(GoDiscoveryExtInfo_Content_(msg)->properties))
    {
        return kNULL;
    }

    return (GoDiscoveryProperty*)kArrayList_At(GoDiscoveryExtInfo_Content_(msg)->properties, index);
}
