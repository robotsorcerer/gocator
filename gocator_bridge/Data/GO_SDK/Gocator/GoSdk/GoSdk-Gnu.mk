ifndef verbose
	SILENT = @
endif

ifndef CC
	CC = gcc
endif

ifndef CXX
	CXX = g++
endif

ifndef config
	config = Debug
endif

ifeq ($(config),Debug)
	TARGET := ../../lib/gnud/libGoSdk.so
	COMPILERFLAGS := -g -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value
	INCLUDEDIRS := -I../../Platform/kApi -I../../Gocator/GoSdk
	DEFINES := -DK_DEBUG -DGO_EMIT
	LINKERFLAGS := -shared -Wl,-rpath,../../lib/gnud
	LIBDIRS := -L../../lib/gnud
	LIBS := -lc -lpthread -lrt -lkApi -lm
	LDFLAGS := $(LINKERFLAGS) $(LIBS) $(LIBDIRS)
	CFLAGS := -fpic $(COMPILERFLAGS) $(DEFINES) $(INCLUDEDIRS)
endif

ifeq ($(config),Release)
	TARGET := ../../lib/gnu/libGoSdk.so
	COMPILERFLAGS := -O2 -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value
	INCLUDEDIRS := -I../../Platform/kApi -I../../Gocator/GoSdk
	DEFINES := -DGO_EMIT
	LINKERFLAGS := -shared -Wl,-rpath,../../lib/gnu
	LIBDIRS := -L../../lib/gnu
	LIBS := -lc -lpthread -lrt -lkApi -lm
	LDFLAGS := $(LINKERFLAGS) $(LIBS) $(LIBDIRS)
	CFLAGS := -fpic $(COMPILERFLAGS) $(DEFINES) $(INCLUDEDIRS)
endif

C_SOURCES = GoSdk/GoSdkLib.c \
	GoSdk/GoSdkDef.c \
	GoSdk/GoLayout.c \
	GoSdk/GoMaterial.c \
	GoSdk/GoMultiplexBank.c \
	GoSdk/GoPartDetection.c \
	GoSdk/GoPartMatching.c \
	GoSdk/GoPartModel.c \
	GoSdk/GoProfileGeneration.c \
	GoSdk/GoSensor.c \
	GoSdk/GoSensorInfo.c \
	GoSdk/GoSetup.c \
	GoSdk/GoSurfaceGeneration.c \
	GoSdk/GoSystem.c \
	GoSdk/GoTransform.c \
	GoSdk/GoUtils.c \
	GoSdk/Internal/GoControl.c \
	GoSdk/Internal/GoDiscovery.c \
	GoSdk/Internal/GoReceiver.c \
	GoSdk/Internal/GoSerializer.c \
	GoSdk/Messages/GoDataSet.c \
	GoSdk/Messages/GoDataTypes.c \
	GoSdk/Messages/GoDiscoveryExtInfo.c \
	GoSdk/Messages/GoHealth.c \
	GoSdk/Outputs/GoOutput.c \
	GoSdk/Outputs/GoAnalog.c \
	GoSdk/Outputs/GoDigital.c \
	GoSdk/Outputs/GoEthernet.c \
	GoSdk/Outputs/GoSerial.c \
	GoSdk/Tools/GoMeasurement.c \
	GoSdk/Tools/GoMeasurements.c \
	GoSdk/Tools/GoTool.c \
	GoSdk/Tools/GoExtParam.c \
	GoSdk/Tools/GoExtTool.c \
	GoSdk/Tools/GoTools.c \
	GoSdk/Tools/GoProfileTools.c \
	GoSdk/Tools/GoProfileToolUtils.c \
	GoSdk/Tools/GoRangeTools.c \
	GoSdk/Tools/GoSurfaceTools.c \
	GoSdk/Tools/GoSurfaceToolUtils.c

CPP_SOURCES = 

.PHONY: all clean

all: $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

OBJECTS = $(C_SOURCES:.c=.o) $(CPP_SOURCES:.cpp=.o)

$(TARGET): $(OBJECTS)
	@echo Linking $(TARGET)
	$(SILENT) $(CXX) $(OBJECTS) $(LDFLAGS) -o$(TARGET)

.c.o:
	@echo Compiling $@
	$(SILENT) $(CC) $(CFLAGS) -c $*.c -o $@

.cpp.o:
	@echo Compiling $@
	$(SILENT) $(CXX) $(CFLAGS) -c $*.cpp -o $@

