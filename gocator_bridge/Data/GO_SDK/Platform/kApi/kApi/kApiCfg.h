/** 
 * @file    kApiCfg.h
 * @brief   Architecture/compiler-specific definitions.
 * 
 * @internal
 * Copyright (C) 2005-2015 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_API_CFG_H
#define K_API_API_CFG_H

/* Detect the compiler family; fall back to GCC as default. */
#if defined(_MSC_VER)
#   define K_MSVC
#elif defined(__TI_COMPILER_VERSION__)
#   define K_TI_CCS
#else
#   define K_GCC
#endif

/* Detect the OS; fall back to non-specific POSIX as default. */
#if defined(_WIN32) || defined(_WIN64)
#   define K_WINDOWS
#elif defined(K_TI_CCS)
#   define K_DSP_BIOS
#elif defined(_WRS_KERNEL)
#   define K_VX_KERNEL
#else
#   define K_POSIX
#   if defined(__linux__)
#       define K_LINUX
#   endif
#   if defined(__APPLE__)
#       define K_DARWIN
#   endif
#   if defined(__QNXNTO__)
#       define K_QNX
#   endif
#endif

#if defined(K_DEBUG)
#   define K_DEBUG_ENABLED          (1)
#else
#   define K_DEBUG_ENABLED          (0)
#endif

#if defined(K_PROFILE)
#   define K_PROFILE_ENABLED        (1)
#else
#   define K_PROFILE_ENABLED        (0)
#endif

/* 
 * Include some C standard headers that we heavily rely on. This list is subject to change; 
 * dependent code should not assume that these headers will always be included here.
 */
#if defined(K_MSVC) && defined(K_DEBUG)
#   define _CRTDBG_MAP_ALLOC
#   include <stdlib.h>
#   include <crtdbg.h>
#else
#   include <stdlib.h>
#endif

#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

/* 
 * Detect pointer size; raise an eror if the pointer size cannot be detected. The behaviour 
 * can be overridden by defining K_POINTER_SIZE as a compiler flag. 
 */
#if !defined(K_POINTER_SIZE)
#   if defined(_WIN64) || defined(WIN64)
#      define K_POINTER_SIZE (8)
#   elif defined(_WIN32) || defined(WIN32)
#      define K_POINTER_SIZE (4)
#   elif defined(__SIZEOF_POINTER__)
#      define K_POINTER_SIZE (__SIZEOF_POINTER__)
#   elif defined(__LP64__) || defined(__LLP64__) || defined(__SILP64__)
#      define K_POINTER_SIZE (8)
#   elif defined(__LP32__) || defined(__ILP32__)
#      define K_POINTER_SIZE (4)
#   elif defined(_TMS320C6X)
#      define K_POINTER_SIZE (4)
#   else
#     error "Pointer size not detected; define K_POINTER_SIZE as compiler flag."
#   endif
#endif

/* 
 * Detect endianness; fall back to little endian as default. The behaviour can be overridden 
 * by defining K_ENDIANNESS as a compiler flag. 
 */
#if !defined(K_ENDIANNESS)
#   if defined(K_GCC) && defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#       define K_ENDIANNESS             (kENDIANNESS_BIG)
#   else 
#       define K_ENDIANNESS             (kENDIANNESS_LITTLE)
#   endif
#endif

/* Define primative data types and calling conventions. */
#if defined(K_MSVC)

#   define kxCall                           __stdcall
#   define kxDlCall                         __cdecl

#   define kExportFx(TYPE)                  __declspec(dllexport) TYPE kCall
#   define kImportFx(TYPE)                  __declspec(dllimport) TYPE kCall

#   define kExportCx(TYPE)                  __declspec(dllexport) TYPE kxDlCall
#   define kImportCx(TYPE)                  __declspec(dllimport) TYPE kxDlCall

#   define kInFx(TYPE)                      TYPE kCall
#   define kInCx(TYPE)                      TYPE kCall

#   define kExportDx(TYPE)                  __declspec(dllexport) TYPE
#   define kImportDx(TYPE)                  __declspec(dllimport) TYPE

#   define kInDx(TYPE)                      TYPE

    typedef unsigned __int8                 kx8u;                       
    typedef unsigned __int16                kx16u;                      
    typedef unsigned __int32                kx32u;                      
    typedef unsigned __int64                kx64u;                      
    typedef __int8                          kx8s;                       
    typedef __int16                         kx16s;                      
    typedef __int32                         kx32s;                      
    typedef __int64                         kx64s;                      
    typedef float                           kx32f;                      
    typedef double                          kx64f;      
    typedef char                            kxChar;                
    typedef unsigned char                   kxByte;                     

#   define kx64U(CONSTANT)                  (CONSTANT##ui64)
#   define kx64S(CONSTANT)                  (CONSTANT##i64)

#elif defined(K_TI_CCS)

#   define kxCall 
#   define kxDlCall

#   define kExportFx(TYPE)                  TYPE kCall
#   define kImportFx(TYPE)                  TYPE kCall

#   define kExportCx(TYPE)                  TYPE kDlCall
#   define kImportCx(TYPE)                  TYPE kDlCall

#   define kInFx(TYPE)                      TYPE kCall
#   define kInCx(TYPE)                      TYPE kCall

#   define kExportDx(TYPE)                  TYPE
#   define kImportDx(TYPE)                  TYPE

#   define kInDx(TYPE)                      TYPE

    typedef unsigned char                   kx8u;                      
    typedef unsigned short                  kx16u;                     
    typedef unsigned int                    kx32u;                     
    typedef unsigned long long              kx64u;                     
    typedef signed char                     kx8s;                      
    typedef signed short                    kx16s;                     
    typedef signed int                      kx32s;                     
    typedef signed long long                kx64s;                     
    typedef float                           kx32f;                     
    typedef double                          kx64f;                     
    typedef char                            kxChar;                
    typedef unsigned char                   kxByte;                    

#   define kx64U(CONSTANT)                  (CONSTANT##LLU)
#   define kx64S(CONSTANT)                  (CONSTANT##LL)

#else

#   define kxCall 
#   define kxDlCall

#   define kExportFx(TYPE)                  TYPE kCall
#   define kImportFx(TYPE)                  TYPE kCall

#   define kExportCx(TYPE)                  TYPE kDlCall
#   define kImportCx(TYPE)                  TYPE kDlCall

#   define kInFx(TYPE)                      TYPE kCall
#   define kInCx(TYPE)                      TYPE kCall

#   define kExportDx(TYPE)                  TYPE
#   define kImportDx(TYPE)                  TYPE

#   define kInDx(TYPE)                      TYPE

    typedef unsigned char                   kx8u;                      
    typedef unsigned short                  kx16u;                     
    typedef unsigned int                    kx32u;                     
    typedef unsigned long long              kx64u;                     
    typedef signed char                     kx8s;                      
    typedef signed short                    kx16s;                     
    typedef signed int                      kx32s;                     
    typedef signed long long                kx64s;                     
    typedef float                           kx32f;                     
    typedef double                          kx64f;                     
    typedef char                            kxChar;                
    typedef unsigned char                   kxByte;                    

#   define kx64U(CONSTANT)                  (CONSTANT##LLU)
#   define kx64S(CONSTANT)                  (CONSTANT##LL)

#endif

#if (K_POINTER_SIZE == 4)

typedef kx32u                               kxSize;                  
#   define kxSIZE_MAX                       k32U_MAX

typedef kx32s                               kxSSize;                   
#   define kxSSIZE_MIN                      k32S_MIN
#   define kxSSIZE_MAX                      k32S_MAX

#elif (K_POINTER_SIZE == 8)

typedef kx64u                               kxSize;                  
#   define kxSIZE_MAX                       k64U_MAX

typedef kx64s                               kxSSize;                   
#   define kxSSIZE_MIN                      k64S_MIN
#   define kxSSIZE_MAX                      k64S_MAX

#endif

#define kALIGN_ANY                          (3)
#define kALIGN_ANY_SIZE                     (1 << kALIGN_ANY)

#define kVarArgList                         va_list

#if defined(K_MSVC) || defined(K_TI_CCS)
#   define kVarArgList_Start_(ARG_PTR, PREV_PARAM)        va_start(ARG_PTR, PREV_PARAM)
#   define kVarArgList_End_(ARG_PTR)                      va_end(ARG_PTR)
#   define kVarArgList_Copy_(ARG_PTR, SOURCE)             ((ARG_PTR) = (SOURCE))
#   define kVarArgList_Next_(ARG_PTR, TYPE)               va_arg(ARG_PTR, TYPE)
#else
#   define kVarArgList_Start_(ARG_PTR, PREV_PARAM)        va_start(ARG_PTR, PREV_PARAM)
#   define kVarArgList_End_(ARG_PTR)                      va_end(ARG_PTR)
#   define kVarArgList_Copy_(ARG_PTR, SOURCE)             va_copy(ARG_PTR, SOURCE)
#   define kVarArgList_Next_(ARG_PTR, TYPE)               va_arg(ARG_PTR, TYPE)
#endif

/*
 * The kBeginHeader/kEndHeader macros are used to wrap header files. This enables any required 
 * definitions, pragmas, etc., to be pushed/popped as part of header processing.
 */
#if defined (__cplusplus)
#   define kBeginHeader()      extern "C" {
#   define kEndHeader()        }
#else
#   define kBeginHeader()
#   define kEndHeader()
#endif

/* 
 * Some source files require platform library headers to be included.  And, at least
 * for Windows, there can sometimes exist complicated rules about the particular order
 * in which headers have to be included. The K_PLATFORM symbol helps to deal with 
 * these issues. 
 * 
 * Any kApi source file that requires platform headers should have #define K_PLATFORM as the 
 * first line in the source file. This ensures that the most common platform headers are 
 * included here, in the correct order. 
 */
#if defined(K_PLATFORM)
#   if defined(K_WINDOWS)
#       include <winsock2.h>
#       include <ws2tcpip.h>
#       include <iphlpapi.h>
#       include <windows.h>
#       include <process.h>
#   endif
#   if defined(K_DSP_BIOS)
#       include <std.h>
#       include <gbl.h>
#       include <clk.h>
#       include <tsk.h>
#       include <lck.h>
#       include <sem.h>
#       include <prd.h>
#       include <netmain.h>
#       include <nettools/nettools.h>
#       include <nettools/inc/inet.h>
#   endif
#   if defined(K_VX_KERNEL)
#       include <vxWorks.h>
#       include <sysLib.h> 
#       include <kernelLib.h> 
#       include <intLib.h> 
#       include <taskLib.h> 
#       include <stdioLib.h> 
#       include <strLib.h>
#       include <sockLib.h> 
#       include <inetLib.h> 
#       include <ioLib.h>
#       include <fioLib.h> 
#       include <selectLib.h> 
#       include <netinet/in.h>
#       include <netinet/tcp.h>
#       include <wrapper/wrapperHostLib.h> 
#       include <netdb.h>
#       include <sys/socket.h>
#       include <net/if.h>
#       include <ipnet/ipioctl.h>
#   endif
#   if defined(K_POSIX)
#       include <errno.h>
#       include <unistd.h>
#       include <pthread.h>
#       include <semaphore.h>
#       include <sys/types.h>
#       include <sys/stat.h>
#       include <sys/fcntl.h>
#       include <sys/socket.h>
#       include <sys/select.h>
#       include <sys/ioctl.h>
#       include <sys/time.h>
#       include <sys/timeb.h>
#       include <netinet/in.h>
#       include <netinet/tcp.h>
#       include <dlfcn.h>
#       include <dirent.h>
#       include <net/if.h>
#       include <netdb.h>
#   endif
#   if defined(K_DARWIN)
#       include <ifaddrs.h>
#       include <mach-o/dyld.h>
#       include <libkern/OSAtomic.h>
#   endif
#endif

#if defined(K_VX_KERNEL)
#   include <base/b_atomic_t.h>

    typedef atomic32Val_t                   kxAtomic32s; 
    typedef atomicVal_t                     kxAtomicPointer; 
#else
    typedef volatile kx32s                  kxAtomic32s; 
    typedef void* volatile                  kxAtomicPointer; 
#endif

#if defined(K_PLATFORM)
#   if defined(K_WINDOWS)
#       define K_OS_INFINITE    INFINITE
        typedef DWORD kThreadId; 
#   elif defined(K_DSP_BIOS)
#       define K_OS_INFINITE    SYS_FOREVER
        typedef TSK_Handle kThreadId; 
#   elif defined(K_VX_KERNEL)
#       define K_OS_INFINITE    WAIT_FOREVER 
        typedef TASK_ID kThreadId; 
#   elif defined(K_POSIX)
#       define K_OS_INFINITE    kINFINITE         /* no special "infinite" value */
        typedef pthread_t kThreadId; 
#   endif
#endif

#if defined(K_COMPAT_5)

/*  Simple renaming (handled by porting script). */
#   define KAPI_ALIGN_SHIFT            kALIGN_ANY
#   define KAPI_ALIGN                  kALIGN_ANY_SIZE

/*
 * These flags probably shouldn't exist; but since they're going to be 
 * required for some time (FS 5 compatibility), we may as well use an 
 * up-to-date style. 
 */
#   if defined(K_DSP)
#       define KAPI_DSP
#   elif defined(K_HOST)
#       define KAPI_HOST
#   elif defined(K_VIRTUAL)
#       define KAPI_VIRTUAL
#   endif

#   if defined(K_WINDOWS)
#       define KAPI_MS_WINDOWS
#   elif defined(K_DSP_BIOS)
#       define KAPI_TI_DSP_BIOS
#   endif

#endif

#endif
