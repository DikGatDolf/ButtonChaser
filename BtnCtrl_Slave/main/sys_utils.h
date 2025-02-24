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
//char * nextWord(char * curWord, bool terminate);
//char * nextWord_delim(char * curWord, bool terminate, char delim);
uint8_t hexToByte(char * hexDigits);
uint8_t charToNibble(char hexDigit);

char * str_trim_l(const char *str);
char * str_next_word(const char *str);

uint8_t crc8_str(uint8_t crc_start, const char *str);
uint8_t crc8_str_n(uint8_t crc_start, const uint8_t *data, size_t len);
uint8_t crc8(uint8_t crc_start, uint8_t data);

bool isNaturalNumberStr(char * str);
bool isFloatStr(char * str);

#undef EXT
#endif /* __SYSUTILS_H__ */

/****************************** END OF FILE **********************************/
