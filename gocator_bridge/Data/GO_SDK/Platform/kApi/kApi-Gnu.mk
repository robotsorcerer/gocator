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
	TARGET := ../../lib/gnud/libkApi.so
	COMPILERFLAGS := -g -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value
	INCLUDEDIRS := -I../../Platform/kApi
	DEFINES := -DK_DEBUG -DK_EMIT -DK_PLUGIN
	LINKERFLAGS := -shared -Wl,-rpath,../../lib/gnud
	LIBDIRS :=
	LIBS := -lc -lpthread -lrt -lm -ldl
	LDFLAGS := $(LINKERFLAGS) $(LIBS) $(LIBDIRS)
	CFLAGS := -fpic $(COMPILERFLAGS) $(DEFINES) $(INCLUDEDIRS)
endif

ifeq ($(config),Release)
	TARGET := ../../lib/gnu/libkApi.so
	COMPILERFLAGS := -O2 -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value
	INCLUDEDIRS := -I../../Platform/kApi
	DEFINES := -DK_EMIT -DK_PLUGIN
	LINKERFLAGS := -shared -Wl,-rpath,../../lib/gnu
	LIBDIRS :=
	LIBS := -lc -lpthread -lrt -lm -ldl
	LDFLAGS := $(LINKERFLAGS) $(LIBS) $(LIBDIRS)
	CFLAGS := -fpic $(COMPILERFLAGS) $(DEFINES) $(INCLUDEDIRS)
endif

C_SOURCES = kApi/kAlloc.c \
	kApi/kApiDef.c \
	kApi/kApiLib.c \
	kApi/kAssembly.c \
	kApi/kObject.c \
	kApi/kType.c \
	kApi/kValue.c \
	kApi/Data/kArray1.c \
	kApi/Data/kArray2.c \
	kApi/Data/kArray3.c \
	kApi/Data/kArrayList.c \
	kApi/Data/kBox.c \
	kApi/Data/kBytes.c \
	kApi/Data/kCollection.c \
	kApi/Data/kImage.c \
	kApi/Data/kList.c \
	kApi/Data/kMath.c \
	kApi/Data/kMap.c \
	kApi/Data/kString.c \
	kApi/Data/kQueue.c \
	kApi/Data/kXml.c \
	kApi/Io/kDat5Serializer.c \
	kApi/Io/kDat6Serializer.c \
	kApi/Io/kDirectory.c \
	kApi/Io/kFile.c \
	kApi/Io/kHttpServer.c \
	kApi/Io/kHttpServerChannel.c \
	kApi/Io/kHttpServerRequest.c \
	kApi/Io/kHttpServerResponse.c \
	kApi/Io/kMemory.c \
	kApi/Io/kNetwork.c \
	kApi/Io/kPath.c \
	kApi/Io/kSerializer.c \
	kApi/Io/kStream.c \
	kApi/Io/kSocket.c \
	kApi/Io/kTcpClient.c \
	kApi/Io/kTcpServer.c \
	kApi/Io/kUdpClient.c \
	kApi/Threads/kAtomic.c \
	kApi/Threads/kLock.c \
	kApi/Threads/kMsgQueue.c \
	kApi/Threads/kPeriodic.c \
	kApi/Threads/kThread.c \
	kApi/Threads/kTimer.c \
	kApi/Threads/kSemaphore.c \
	kApi/Utils/kBackTrace.c \
	kApi/Utils/kDebugAlloc.c \
	kApi/Utils/kDynamicLib.c \
	kApi/Utils/kEvent.c \
	kApi/Utils/kObjectPool.c \
	kApi/Utils/kPlugin.c \
	kApi/Utils/kPoolAlloc.c \
	kApi/Utils/kUserAlloc.c \
	kApi/Utils/kUtils.c

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

