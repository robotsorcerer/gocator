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
	TARGET := ../../bin/gnud/GoSystemExample
	COMPILERFLAGS := -g -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value
	INCLUDEDIRS := -I../../Platform/kApi -I../../Gocator/GoSdk
	DEFINES :=
	LINKERFLAGS := -Wl,-rpath,../../lib/gnud
	LIBDIRS := -L../../lib/gnud
	LIBS := -lkApi -lGoSdk
	LDFLAGS := $(LINKERFLAGS) $(LIBS) $(LIBDIRS)
	CFLAGS := -fpic $(COMPILERFLAGS) $(DEFINES) $(INCLUDEDIRS)
endif

ifeq ($(config),Release)
	TARGET := ../../bin/gnu/GoSystemExample
	COMPILERFLAGS := -O2 -std=gnu99 -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-value
	INCLUDEDIRS := -I../../Platform/kApi -I../../Gocator/GoSdk
	DEFINES :=
	LINKERFLAGS := -Wl,-rpath,../../lib/gnu
	LIBDIRS := -L../../lib/gnu
	LIBS := -lkApi -lGoSdk
	LDFLAGS := $(LINKERFLAGS) $(LIBS) $(LIBDIRS)
	CFLAGS := -fpic $(COMPILERFLAGS) $(DEFINES) $(INCLUDEDIRS)
endif

C_SOURCES = GoSystemExample/GoSystemExample.c

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

