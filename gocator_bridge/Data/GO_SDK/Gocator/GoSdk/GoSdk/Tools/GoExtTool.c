/** 
 * @file    GoExtTool.c
 *
 * @internal
 * Copyright (C) 2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoExtTool.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClass(Go, GoExtTool, GoTool)
    kAddVMethod(GoExtTool, kObject, VRelease)
    kAddVMethod(GoExtTool, GoTool, VInit)
    kAddVMethod(GoExtTool, GoTool, VRead)
    kAddVMethod(GoExtTool, GoTool, VWrite)
kEndClass()

GoFx(kStatus) GoExtTool_Construct(GoExtTool* tool, kObject sensor, kAlloc allocator)
{
    return GoExtTool_VInit(tool, kTypeOf(GoExtTool), sensor, allocator);
}

GoFx(kStatus) GoExtTool_VInit(GoExtTool tool, kType type, kObject sensor, kAlloc alloc)
{
    GoExtToolClass* obj = tool; 
    kSize i;

    kCheck(GoTool_Init(tool, type, GO_TOOL_EXTENSIBLE, sensor, alloc)); 
    kInitFields_(GoExtTool, tool);
            
    obj->source = GO_DATA_SOURCE_TOP; 
    obj->anchoring[0].anchor = -1;
    obj->anchoring[1].anchor = -1;
    obj->anchoring[2].anchor = -1;

    for (i = 0; i < 3; i++)
    {
        obj->anchoring[i].anchor = -1;
        kCheck(kArrayList_Construct(&obj->anchoring[i].options, kTypeOf(k32u), 0, alloc));
    }
    
    kCheck(kArrayList_Construct(&obj->sourceOptions, kTypeOf(GoDataSource), 0, alloc));
    kCheck(kArrayList_Construct(&obj->parameters, kTypeOf(GoExtParam), 0, alloc));
    
    return kOK; 
}

GoFx(kStatus) GoExtTool_VRelease(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);
    kSize i;

    kCheck(kDisposeRef(&obj->sourceOptions));
    
    for (i = 0; i < 3; i++)
    {
        kCheck(kDisposeRef(&obj->anchoring[i].options));
    }
    
    kCheck(kDisposeRef(&obj->parameters));

    return GoTool_VRelease(tool);
}

GoFx(kStatus) GoExtTool_VRead(GoExtTool tool, kXml xml, kXmlItem item)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);
    kXmlItem tempItem = kNULL;
    kXmlItem anchorItem = kNULL;
    kXmlItem paramItem = kNULL;
    kXmlItem measurementOptionsItem = kNULL;
    kText64 currentVal;
    kText256 tempText;
    kSize i;
    kStatus exception;
    const kChar* dimensions[3] = {"X", "Y", "Z"};
    
    kCheck(GoTool_VRead(tool, xml, item)); 

    kCheck(kXml_AttrText(xml, item, "version", obj->version, 32));
    kCheck(kXml_AttrText(xml, item, "type", obj->toolType, 128));
    
    kCheck(!kIsNull(anchorItem = kXml_Child(xml, item, "Anchor")));

    for (i = 0; i < kXml_ChildCount(xml, anchorItem); i++)
    {
        kCheck(kXml_Child32s(xml, anchorItem, dimensions[i], &obj->anchoring[i].anchor));
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, dimensions[i])));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->anchoring[i].used));
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));         
        kCheck(GoOptionList_ParseList32u(tempText, obj->anchoring[i].options));  
    }
    
    kCheck(kXml_Child32s(xml, item, "Source", &obj->source));
    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "Source")));
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText))); 
    kCheck(GoOptionList_ParseList32u(tempText, obj->sourceOptions));  

    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "Parameters")));

    for (i = 0; i < kXml_ChildCount(xml, tempItem); i++)
    {
        GoExtParam param = kNULL;
        kType paramType;
        kBool insertParam = kFALSE;

        kTry
        {
            paramItem = kXml_ChildAt(xml, tempItem, i);

            if (i >= kArrayList_Count(obj->parameters)
                || kIsNull(param = kArrayList_As_(obj->parameters, i, GoExtParam)))
            {
                if (!kIsNull(param))
                {
                    kTest(kArrayList_Remove(obj->parameters, i, &param));
                    kDisposeRef(&param);
                }

                kTest(kXml_AttrText(xml, paramItem, "type", currentVal, kCountOf(currentVal)));
                kTestArgs(!kIsNull(paramType = GoExtUtils_GetKType(currentVal)));
                kTest(GoExtParam_Construct(&param, paramType, obj->base.sensor, kObject_Alloc(tool)));
                insertParam = kTRUE;
            }

            kTest(GoExtParam_Read(param, xml, paramItem));
        
            if (insertParam)
            {
                kTest(kArrayList_Insert(obj->parameters, i, &param));
            }
        }
        kCatch(&exception)
        {
            kDestroyRef(&param);
            kEndCatch(exception);
        }
    }
    
    return kOK;
}

GoFx(kStatus) GoExtTool_VWrite(GoExtTool tool, kXml xml, kXmlItem item)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool); 
    kXmlItem tempItem;
    kSize i;
    const kChar* dimensions[3] = {"X", "Y", "Z"};

    //custom configuration nodes    
    kCheck(GoTool_VWrite(tool, xml, item)); 

    kCheck(kXml_SetAttrText(xml, item, "type", obj->toolType));
    kCheck(kXml_SetAttrText(xml, item, "version", obj->version));
    kCheck(kXml_SetChild32s(xml, item, "Source", obj->source));
    
    kCheck(kXml_AddItem(xml, item, "Anchor", &tempItem));
    for (i = 0; i < 3; i++)
    {
        if (obj->anchoring[i].used)
        {
            kCheck(kXml_SetChild32s(xml, tempItem, dimensions[i], obj->anchoring[i].anchor));
        }
    }

    kCheck(kXml_AddItem(xml, item, "Parameters", &tempItem));

    for (i = 0; i < kArrayList_Count(obj->parameters); i++)
    {
        GoExtParam param = *(GoExtParam*)kArrayList_At(obj->parameters, i);
        kXmlItem valueItem = kNULL;

        kCheck(kXml_AddItem(xml, tempItem, GoExtParam_Id(param), &valueItem));
        kCheck(GoExtParam_Write(param, xml, valueItem));
    }

    return kOK;
}

GoFx(kStatus) GoExtTool_SetDisplayName(GoExtTool tool, const kChar* value)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    return GoTool_SetName(tool, value);
}

GoFx(const kChar*) GoExtTool_DisplayName(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    return obj->base.name;
}

GoFx(const kChar*) GoExtTool_Type(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    return obj->toolType;
}

GoFx(kBool) GoExtTool_XAnchorSupportEnabled(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    return obj->anchoring[0].used;
}

GoFx(kBool) GoExtTool_YAnchorSupportEnabled(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    return obj->anchoring[1].used;
}

GoFx(kBool) GoExtTool_ZAnchorSupportEnabled(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    return obj->anchoring[2].used;
}

GoFx(kSize) GoExtTool_MeasurementCount(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool); 

    return kArrayList_Count(obj->base.measurements);
}

GoFx(GoMeasurement) GoExtTool_MeasurementAt(GoExtTool tool, kSize index)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    if (index > kArrayList_Count(obj->base.measurements))
    {
        return kNULL;
    }

    return *(GoMeasurement*)kArrayList_At(obj->base.measurements, index);
}

GoFx(GoMeasurement) GoExtTool_FindMeasurementByName(GoExtTool tool, const kChar* name)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->base.measurements); i++)
    {
        GoMeasurementClass* meas = (GoMeasurementClass*)kArrayList_At(obj->base.measurements, i);

        //not quite right - this checks the measurement name, but not the extensible tool's measurement type name identifier
        if (strcmp(name, meas->name) == 0)
        {
            return *(GoMeasurement*)kArrayList_At(obj->base.measurements, i);
        }
    }

    return kNULL;
}

GoFx(kStatus) GoExtTool_SetSource(GoExtTool tool, GoDataSource source)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));
    kCheck(GoOptionList_Check32u(kArrayList_Data(obj->sourceOptions), kArrayList_Count(obj->sourceOptions), source));
    obj->source = source;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoExtTool_SourceOptionCount(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->sourceOptions);
}

GoFx(k32u) GoExtTool_SourceOptionAt(GoExtTool tool, kSize index)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->sourceOptions));

    return kArrayList_As_(obj->sourceOptions, index, k32u);
}

GoFx(GoDataSource) GoExtTool_Source(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->source;
}

GoFx(kSize) GoExtTool_XAnchorOptionCount(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->anchoring[0].options);
}

GoFx(k32u) GoExtTool_XAnchorOptionAt(GoExtTool tool, kSize index)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->anchoring[0].options));

    return kArrayList_As_(obj->anchoring[0].options, index, k32u);
}

GoFx(k32s) GoExtTool_XAnchor(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring[0].anchor;
}

GoFx(kStatus) GoExtTool_SetXAnchor(GoExtTool tool, k32s id)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id >= 0)
    {
        kCheck(GoOptionList_Check32u(kArrayList_Data(obj->anchoring[0].options), kArrayList_Count(obj->anchoring[0].options), id));
    }

    obj->anchoring[0].anchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoExtTool_XAnchorEnabled(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->anchoring[0].anchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoExtTool_YAnchorOptionCount(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->anchoring[1].options);
}

GoFx(k32u) GoExtTool_YAnchorOptionAt(GoExtTool tool, kSize index)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->anchoring[1].options));

    return kArrayList_As_(obj->anchoring[1].options, index, k32u);
}

GoFx(k32s) GoExtTool_YAnchor(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring[1].anchor;
}

GoFx(kStatus) GoExtTool_SetYAnchor(GoExtTool tool, k32s id)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id >= 0)
    {
        kCheck(GoOptionList_Check32u(kArrayList_Data(obj->anchoring[1].options), kArrayList_Count(obj->anchoring[1].options), id));
    }

    obj->anchoring[1].anchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoExtTool_YAnchorEnabled(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->anchoring[1].anchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoExtTool_ZAnchorOptionCount(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->anchoring[2].options);
}

GoFx(k32u) GoExtTool_ZAnchorOptionAt(GoExtTool tool, kSize index)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->anchoring[2].options));

    return kArrayList_As_(obj->anchoring[2].options, index, k32u);
}

GoFx(kStatus) GoExtTool_SetZAnchor(GoExtTool tool, k32s id)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id >= 0)
    {
        kCheck(GoOptionList_Check32u(kArrayList_Data(obj->anchoring[2].options), kArrayList_Count(obj->anchoring[2].options), id));
    }

    obj->anchoring[2].anchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}


GoFx(kBool) GoExtTool_ZAnchorEnabled(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->anchoring[2].anchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(k32s) GoExtTool_ZAnchor(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring[2].anchor;
}

GoFx(kSize) GoExtTool_ParameterCount(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->parameters);
}

GoFx(GoExtParam) GoExtTool_ParameterAt(GoExtTool tool, kSize index)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (index > kArrayList_Count(obj->parameters))
    {
        return kNULL;
    }

    return *(GoExtParam*)kArrayList_At(obj->parameters, index);
}

GoFx(GoExtParam) GoExtTool_FindParameterById(GoExtTool tool, const kChar* id)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);
    kSize i;

    GoSensor_SyncConfig(obj->base.sensor);

    for (i = 0; i < kArrayList_Count(obj->parameters); i++)
    {
        GoExtParam value = *(GoExtParam*)kArrayList_At(obj->parameters, i);

        if (strcmp(id, GoExtParam_Id(value)) == 0)
        {
            return value;
        }
    }

    return kNULL;
}

GoFx(const kChar*) GoExtTool_Version(GoExtTool tool)
{
    GoExtToolClass* obj = GoExtTool_Cast_(tool);

    return obj->version;
}
