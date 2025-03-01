/*****************************************************************************

sys_utils.h

Include file for sys_utils.c

******************************************************************************/
#ifndef __SYSUTILS_H__
#define __SYSUTILS_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */


#define CRC_8_POLYNOMIAL 0x8C

#ifndef NULL_PTR
#define NULL_PTR (void*)0
#endif

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define SWOP_U64(x, y) do { uint64_t(x) _z = x; x = y; y = _z; } while(0)
#define SWOP_U32(x, y) do { uint32_t(x) _z = x; x = y; y = _z; } while(0)
#define SWOP_U16(x, y) do { uint16_t(x) _z = x; x = y; y = _z; } while(0)
#define SWOP_U8(x, y) do { uint8_t(x) _z = x; x = y; y = _z; } while(0)

/******************************************************************************
Macros
******************************************************************************/

typedef enum
{
    //eTaskIndexIdle = 0,
    // eTaskIndexMain,
    eTaskIndexConsole = 0,
    eTaskIndexRgb,
    // eTaskIndexBattery,
    // eTaskIndexMotion,
//    eTaskIndexSigfox,

    /* Keep this at the end */
    eTaskIndexMax
}eTaskIndex;

/******************************************************************************
Struct & Unions
******************************************************************************/
typedef struct 
{
    bool init_done;
    void * parameter_to_pass;
    TaskHandle_t handle;
    TaskStatus_t details;
    configSTACK_DEPTH_TYPE stack_depth;
    configSTACK_DEPTH_TYPE stack_unused;
}TaskInfo_t;

extern TaskInfo_t * sys_tasks[];

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/

uint8_t crc8_str(uint8_t crc_start, const char *str);
uint8_t crc8_str_n(uint8_t crc_start, const uint8_t *data, size_t len);
uint8_t crc8(uint8_t crc_start, uint8_t data);

#undef EXT
#endif /* __SYSUTILS_H__ */

/****************************** END OF FILE **********************************/
