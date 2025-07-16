/*******************************************************************************
Module:     dev_nvstore.c
Purpose:    This file contains the Non-Volatile storage functions for the 
             arduino nano, which makes use of the 1kB EEPROM to store data.
Author:     Rudolph van Niekerk

The 8-byte header describes the block of data that follows. The header contains:
  - A magic number to identify the block
  - The length of the data in the block
  - A CRC of the entire block of data
  - The number of times this block has been written to.

At startup, the system will read the EEPROM and check the magic number, and 
find the last block of data that was written to. This will be the block that
is used for reading. Writing will always be done to the next block. The block 
sizes are fixed, to make it easier to manage the EEPROM space and ensure that 
the system can keep accurate track of the number of times a block has been 
written.

The Write count is used to determine 2 things:
1) Which was the last block written to (i.e. which data is current)
2) How many times the EEPROM has been written to. This is important because the
   EEPROM has a limited number of write cycles before it will fail. By keeping
   track of the number of writes, we can determine when the EEPROM is nearing 
   the end of its life.

The first block written will be 0, the 2nd block will be 1.... 
 whenthe EEPROM is full and the write operations wrappes to the start, the first 
 block's count will be 64 (1024/block size), the 2nd block will be 65, etc.

 At any given point, the block with the highest write count will be the most
 recent block written to, and its write count/64 will be the number of times that 
 block of EEPROM has been written to.

 By cycling though the entire eeprom, we are creating an inherent "wear levelling"
 mechanism, which should ensure that the EEPROM will last longer than if we were 
 to write to the same block over and over again.

 *******************************************************************************/


/*******************************************************************************
includes
 *******************************************************************************/
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include <EEPROM.h>

#define __NOT_EXTERN__
#include "dev_nvstore.h"
#undef __NOT_EXTERN__

#include "sys_utils.h"
#include "hal_timers.h"
#include "str_helper.h"
#include "dev_console.h"

/*******************************************************************************
Macros and Constants
 *******************************************************************************/
#ifdef PRINTF_TAG
  #undef PRINTF_TAG
#endif
#define PRINTF_TAG ("NVStore") /* This must be undefined at the end of the file*/

/*******************************************************************************
local defines 
 *******************************************************************************/

#define NVSTORE_MAGIC           (0xCAFE)    /* A magic number to identify the block */
#define NVSTORE_BLOCK_SIZE      (16)        /* The size of the block of data, incl the header */
#define NVSTORE_MAX_WR_CYCLES   (100000)    /* The maximum number of write cycles allowed per block */
#define NVSTORE_SIZE            (E2END + 1)      /* The size of the EEPROM on the Nano */
#define NVSTORE_BLOCK_CNT       (NVSTORE_SIZE / NVSTORE_BLOCK_SIZE) /* The number of blocks in the EEPROM */
#define NVSTORE_WR_CNT_MAX      (100000 * NVSTORE_BLOCK_CNT)    /* The maximum number of write cycles in the EEPROM */

#define NVSTORE_DATA_VERSION    (0x00)      /* The current supported version of the data in the block */

//#define NVSTORE_MAGIC_BAD       (0x0BAD)    /* A magic number to identify the block as BAD (several CRC failures?) */

/*******************************************************************************
 Local structure
 *******************************************************************************/
typedef struct
{
    uint16_t magic;         /* A magic number to identify the block */
    uint32_t wr_cnt;        /* The write count of blocks for the entire EEPROM. 
                                The first block written will be 0, the next block will be 1.... when it comes to the first block again, it will be 64 (1024/block size) */
    uint8_t version;        /* Version of the data in the block (defines the data content) */
} nvstore_block_header_t;

#define NVSTORE_BLOCK_DATA_SIZE (NVSTORE_BLOCK_SIZE - sizeof(nvstore_block_header_t) - sizeof(uint8_t))

typedef struct
{
    nvstore_block_header_t hdr;   // Length: 7 bytes          magic #             wr_cnt              version             crc
    uint8_t data[NVSTORE_BLOCK_DATA_SIZE];
    uint8_t crc;            /* A CRC of the entire block of data - IMPORTANT: MUST BE WRITTEN LAST */
} nvstore_block_t;

typedef struct
{
    struct 
    {
        uint8_t     last_rd;
        uint8_t     next_wr;
        uint32_t    wr_cnt;
    } current;
    nvstore_block_t rd_block;
    bool            new_data;
#if (DEV_NVSTORE_DEBUG == 1)
    stopwatch_ms_t sw;
#endif /* DEV_NVSTORE_DEBUG */
} dev_nvstore_t;

/*******************************************************************************
 Local function prototypes
 *******************************************************************************/
uint8_t _nvstore_write_next_block_data(uint8_t * data);
uint8_t _nvstore_read_block(uint8_t * data, uint8_t block_nr);

void _dev_nvstore_menu_handler_print_block(uint8_t flags, uint8_t block_nr, nvstore_block_t * block, uint8_t calc_crc);

#ifdef CONSOLE_ENABLED
#if (DEV_NVSTORE_DEBUG == 1)
void _dev_nvstore_menu_handler_rd(void);
void _dev_nvstore_menu_handler_wr(void);
/*******************************************************************************
 Local variables
 *******************************************************************************/

static console_menu_item_t _nvstore_menu_items[] =
{										     // 01234567890123456789012345678901234567890123456789012345678901234567890123456789
    {"read",    _dev_nvstore_menu_handler_rd,   "Reads Non-Volatile (EEPROM) data"},
    {"write",   _dev_nvstore_menu_handler_wr,   "Writes Non-Volatile (EEPROM) data"},
};
#endif /* DEV_NVSTORE_DEBUG */
#endif /* CONSOLE_ENABLED */

dev_nvstore_t _nvstore;

/*******************************************************************************
 Local (private) Functions
 *******************************************************************************/

uint8_t _nvstore_read_block(uint8_t * data, uint8_t block_nr)
{
    uint8_t block_crc = 0;
    int _rd_addr = block_nr * NVSTORE_BLOCK_SIZE;
    if (block_nr >= NVSTORE_BLOCK_CNT)
    {
        iprintln(trNVSTORE, "Block number out of range: %d (max: %d)", block_nr, NVSTORE_BLOCK_CNT);
        return 0;
    }

    for (int i = 0; i < NVSTORE_BLOCK_SIZE; i++)
    {
        data[i] = EEPROM.read(_rd_addr + i);
        block_crc = crc8(block_crc, data[i]);
    }
    return block_crc; //Should be 0x00
}

uint8_t _nvstore_write_next_block_data(uint8_t * data)
{
    nvstore_block_t _tmp_block;
    uint8_t         block_nr    = _nvstore.current.next_wr;
    int             wr_addr     = block_nr * NVSTORE_BLOCK_SIZE;
    uint8_t       * src = (uint8_t*)&_tmp_block;

    _tmp_block.hdr.magic = NVSTORE_MAGIC;
    _tmp_block.hdr.wr_cnt = _nvstore.current.wr_cnt + 1;
    _tmp_block.hdr.version = NVSTORE_DATA_VERSION;
    memcpy(_tmp_block.data, data, NVSTORE_BLOCK_DATA_SIZE);
    _tmp_block.crc = crc8_n(0, (uint8_t *)&_tmp_block, NVSTORE_BLOCK_SIZE-1);

    iprint(trNVSTORE, "#Writing ");
    _dev_nvstore_menu_handler_print_block(trNVSTORE, block_nr, &_tmp_block, 0x00);

    for (int i = 0; i < NVSTORE_BLOCK_SIZE; i++)
        EEPROM.write(wr_addr+i, src[i]);

    _nvstore.current.next_wr = (block_nr + 1) % NVSTORE_BLOCK_CNT;
    _nvstore.new_data = true;

    iprintln(trNVSTORE, "Next block to write: %d", _nvstore.current.next_wr);

    return block_nr;
}

void _dev_nvstore_menu_handler_print_block(uint8_t flags, uint8_t block_nr, nvstore_block_t * block, uint8_t calc_crc)
{
    uint8_t status = 0;
    if (block->hdr.magic != NVSTORE_MAGIC)
        SET_BIT(status, 0);
    if (calc_crc != 0)
        SET_BIT(status, 1);
    if (block->hdr.version != NVSTORE_DATA_VERSION)
        SET_BIT(status, 2);

    iprint(flags, "NVStore Block %d/%d ", block_nr, NVSTORE_BLOCK_CNT);
    if (block->hdr.magic != NVSTORE_MAGIC)
        iprint(flags, "- UNUSED");
    else if (calc_crc != 0)
        iprint(flags, "- BAD CRC (0x%02X)", block->crc);
    else if (block->hdr.version != NVSTORE_DATA_VERSION)
        iprint(flags, "- Unknown Version (%d)", block->hdr.version);
    else //if (status == 0)
    {
        iprint(flags, "(Wr Cnt: %lu)", block->hdr.wr_cnt/NVSTORE_BLOCK_CNT);
        for (unsigned int i = 0; i < NVSTORE_BLOCK_DATA_SIZE; i++)
            iprint(flags, "%02X ", block->data[i]);
    }
    iprintln(flags, " :");
    console_print_ram(flags, block, 0, NVSTORE_BLOCK_SIZE);
}

#ifdef CONSOLE_ENABLED
#if (DEV_NVSTORE_DEBUG == 1)
void _dev_nvstore_menu_handler_rd(void)
{
    char *argStr;// = console_arg_pop();
    bool help_requested = console_arg_help_found();
    nvstore_block_t _tmp_block;
    uint8_t block_crc = 0;


    if (console_arg_cnt() == 0) // Show the current state of the PWM
    {
        if (_nvstore.current.last_rd >= NVSTORE_BLOCK_CNT)
        {
            iprintln(trNVSTORE, "No valid block exists in NV Store");
        }
        else
        {
            _dev_nvstore_menu_handler_print_block(trALWAYS, _nvstore.current.last_rd, &_nvstore.rd_block, 0x00);
        }        
    }

    //Go through every available argument
    while ((console_arg_cnt() > 0) && (!help_requested))
    {
        uint32_t tmp_block_nr;
        argStr = console_arg_pop();
        /*if ((0 == strcasecmp(argStr, "help")) || (0 == strcasecmp(argStr, "?"))) //is it a ? or help
        {
            help_requested = true; //Disregard the rest of the arguments
            break; //from while
        }
        else */
        if (str2uint32(&tmp_block_nr, argStr, 0))
        {
            //Is the value within range?
            if (tmp_block_nr >= NVSTORE_BLOCK_CNT)
            {
                iprintln(trNVSTORE, "Block number, %d, out of range (max: %d)", tmp_block_nr, NVSTORE_BLOCK_CNT);
                help_requested = true; //Disregard the rest of the arguments
                break; //from while
            }
            //current_block = tmp_block_nr;
            block_crc = _nvstore_read_block((uint8_t *)&_tmp_block, (uint8_t)tmp_block_nr);
            _dev_nvstore_menu_handler_print_block(trALWAYS, tmp_block_nr, &_tmp_block, block_crc);
        }
        else
        {
            help_requested = true;
            iprintln(trALWAYS, "Invalid argument \"%s\"", argStr);
        }
    }
    
    if (!help_requested)
    {
        iprintln(trALWAYS, "\nNext block to write: %d", _nvstore.current.next_wr);
    }

    if (help_requested)
    {
        iprintln(trALWAYS, " Usage: \"read [<block index 0 to %d>]\"", NVSTORE_BLOCK_CNT-1);
#if REDUCE_CODESIZE==0        
        iprintln(trALWAYS, "    <block #>   - A Block number (0-%d)", NVSTORE_BLOCK_CNT-1);
        iprintln(trALWAYS, "    If <block #> is omitted, the current block is read");
        //                //          1         2         3         4         5         6         7         8         9
        //                //0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#endif /* REDUCE_CODESIZE */
    }
}

void _dev_nvstore_menu_handler_wr(void)
{
    char *argStr;// = console_arg_pop();
    bool help_requested = console_arg_help_found();
    uint8_t _tmp_data[NVSTORE_BLOCK_DATA_SIZE];
    unsigned int space_left = NVSTORE_BLOCK_DATA_SIZE;
    unsigned int bytes_to_parse = 0;
    bool buff_full = false;
    char * src;

    while ((console_arg_cnt() > 0) && (!help_requested))
    {
        argStr = console_arg_pop();\

        /*if ((0 == strcasecmp(argStr, "help")) || (0 == strcasecmp(argStr, "?"))) //is it a ? or help
        {
            help_requested = true; //Disregard the rest of the arguments
            break; //from while
        }
        else */
        if ((buff_full) || (space_left <= 0))
        {
            iprintln(trALWAYS, "Buffer is full. Ignoring \"%s\"", argStr);
            continue; // with the next argument
        }
        //Is this a hex value?
        else if (is_hex_str(argStr,0))
        {
            //Remove leading "0x", "x", or "#" if present
            if ((argStr[0] == '0') && ((argStr[1] == 'x') || (argStr[1] == 'X')))
                src = argStr + 2;
            else if ((argStr[0] == 'x') || (argStr[0] == 'X') || (argStr[0] == '#'))
                src = argStr + 1;
            else
                src = argStr;                

            //How many bytes do we need to write?
            bytes_to_parse = strlen(src)/2;
            if (strlen(src) % 2)
                bytes_to_parse++; //In case the string length is odd

            //We check for enough space....
            if  (space_left < bytes_to_parse)
            {
                buff_full = true;  //Cannot parse the current string, not enough space left in the block
                iprintln(trALWAYS, "Rejecting \"%s\", requires %d bytes (only %d left)", argStr, bytes_to_parse, space_left);
                continue; // with the next argument
            }

            //We start from the LSB (far right of the string)
            src += (strlen(src) - 2);
            while (bytes_to_parse > 0)
            {
                //Check if we have a valid hex char in the msb (this is only needed if the string length was odd)
                if (!isxdigit(*src))
                    *src = '0'; //If not, replace it with a '0'
                _tmp_data[NVSTORE_BLOCK_DATA_SIZE-space_left] = hex2byte(src);
                *src = 0;   //Null terminal the string right here
                src -= 2;   //Move src ptr back 2chars (1 byte)
                bytes_to_parse--;   
                space_left--;   //Decrement the space left
            }
            continue;
        }
        //Is this a string?
        else if ((((argStr[0] == '\"') && (argStr[strlen(argStr)-1] == '\"'))   ||
                  ((argStr[0] == '\'') && (argStr[strlen(argStr)-1] == '\'')))  && (strlen(argStr) > 2))
        {
            //Remove quotes from string if present
            argStr[strlen(argStr)-1] = 0;
            src = argStr + 1;

            bytes_to_parse = strlen(src);

            //We check for enough space....
            if  (space_left < bytes_to_parse)
            {
                buff_full = true;  //Cannot parse the current string, not enough space left in the block
                iprintln(trALWAYS, "Rejecting \"%s\", requires %d bytes (only %d left)", src, bytes_to_parse, space_left);
                continue; // with the next argument
            }

            strncpy((char *)&_tmp_data[NVSTORE_BLOCK_DATA_SIZE-space_left], src, space_left);
            space_left -= bytes_to_parse;

            continue; // with the next argument
        }
        //Is this an integer value?
        else if (is_natural_number_str(argStr, 0))
        {
            int64_t _val = 0l;
            int64_t _max_val = INT8_MAX;
            int64_t _min_val = INT8_MIN;
            str2int64(&_val, argStr, 0);
            bytes_to_parse = 1;

            //We check for enough space....
            while (((_val > _max_val) || (_val < _min_val)) && (bytes_to_parse < space_left))
            {
                //Doesn't fit into a n byte(s), so we need to add another byte
                bytes_to_parse++;
                ////Which will increase the range of max and min values
                _max_val = (_max_val << 8) | 0xFF;
                _min_val = (_min_val << 8) & 0x00;
            } 

            if  (space_left < bytes_to_parse)
            {
                buff_full = true;  //Cannot parse the current string, not enough space left in the block
                iprintln(trALWAYS, "Rejecting \"%s\", requires %d bytes (only %d left)", argStr, bytes_to_parse, space_left);
                continue; // with the next argument
            }

            //Great, the number will fit into the space left
            while ((_val != 0) && (space_left > 0))
            {
                _tmp_data[NVSTORE_BLOCK_DATA_SIZE-space_left] = (uint8_t)(_val & 0xFF);
                _val >>= 8;
                space_left--;
            }
            continue; // with the next argument
        }
        //else what is this????
        iprintln(trALWAYS, "Invalid argument \"%s\"", argStr);
        help_requested = true;
        break;
    }

    if (!help_requested)
    {
        //Only do this if we have written something to the buffer
        if (space_left >= NVSTORE_BLOCK_DATA_SIZE)
        {
            help_requested = true;
        }
        else if (!dev_nvstore_write(_tmp_data, NVSTORE_BLOCK_DATA_SIZE-space_left))
        {
            iprintln(trALWAYS, "!! ERROR !! Failed to write data to EEPROM");
        }
    }

    if (help_requested)
    {
        iprintln(trALWAYS, " Usage: \"write [<element_1> <element_2> ... <element_%d>]\"", NVSTORE_BLOCK_DATA_SIZE);
#if REDUCE_CODESIZE==0        
        iprintln(trALWAYS, "    <element> - up to %d data elements as hex values (0x..), strings (\"..\") or integers", NVSTORE_BLOCK_DATA_SIZE);
        // iprintln(trALWAYS, "    <element> - up to %d data elements in any of the following formats:", NVSTORE_BLOCK_DATA_SIZE);
        // iprintln(trALWAYS, "              * hex value strings (preceded '0x...')");
        // iprintln(trALWAYS, "              * string(s) (enclosed in \"...\" or '...'");
        // iprintln(trALWAYS, "              * integer values  (e.g \"11 255 -71 23 1290 91\")");
        iprintln(trALWAYS, "     Multiple elements can be given, seperated by spaces, e.g \"write 11 0xff -7 '23' 1290\"");
        iprintln(trALWAYS, "     Hex and integer data is stored in big-endian format (i.e. LSB to MSB)");
        //                //          1         2         3         4         5         6         7         8         9
        //                //0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#endif /* REDUCE_CODESIZE */
    }
}
#endif /* DEV_NVSTORE_DEBUG */
#endif /* CONSOLE_ENABLED */
/*******************************************************************************
 Global (public) Functions
 *******************************************************************************/

void dev_nvstore_init(void)
{
    //bool found = false;
    nvstore_block_t _tmp_block;
    uint8_t _calc_block_crc = 0;
#if (DEV_NVSTORE_DEBUG == 1)
    char buff[16];

    //iprintln(trNVSTORE, "#Initialising (Version %d, Block Size %d bytes)", NVSTORE_DATA_VERSION, NVSTORE_BLOCK_SIZE);
    sys_stopwatch_ms_start(&_nvstore.sw);
#endif /* DEV_NVSTORE_DEBUG */

    _nvstore.current.last_rd = (uint8_t)-1;     /* Invalid block number */
    _nvstore.current.wr_cnt = 0;                /* Invalid write count */
    _nvstore.current.next_wr = 0;               /* Start writing at the beginning of the EEPROM */
    _nvstore.new_data = false;

    //Start at the beginning of the EEPROM
    for (int block_nr = 0; block_nr < NVSTORE_BLOCK_CNT; block_nr++)
    {
        //Read the block and calculate the crc while we are at it
        _calc_block_crc = _nvstore_read_block((uint8_t *)&_tmp_block, block_nr);

        //Make sure it is legit (Magic Number & the CRC must be correct)
        if (_tmp_block.hdr.magic != NVSTORE_MAGIC)
            break; //from for-loop - we have reached the end of the valid blocks

        //Check if the data is valid (CRC is good)
        if (0 != _calc_block_crc)
        {
            //iprintln(trNVSTORE, "#CRC mismatch in block %d - Calc: 0x%02X (Expected: 0x00)", block_nr, _tmp_block.crc);

            //The data CRC is a invalid, but maybe we want to start writing at the next block (skipping this block for this round)
            _nvstore.current.next_wr = (block_nr+1) % NVSTORE_BLOCK_CNT;

            //Skip this block for all other purposes
            continue;
        }

        //If we have found a valid block before, check which one is more current
        if ((_nvstore.new_data) && (_tmp_block.hdr.wr_cnt < _nvstore.rd_block.hdr.wr_cnt))
        {
            //Means the previous block was the most recent, so we can stop looking
            break; //from for-loop
        }

        //This block is good and "newer" than the previous one
        memcpy(&_nvstore.rd_block, &_tmp_block, NVSTORE_BLOCK_SIZE);
        _nvstore.current.wr_cnt = _tmp_block.hdr.wr_cnt;
        _nvstore.current.last_rd = block_nr;
        _nvstore.current.next_wr = (block_nr+1) % NVSTORE_BLOCK_CNT; //Might need the wrap around to block 0
        _nvstore.new_data = true;
    }

#if (DEV_NVSTORE_DEBUG == 1)
    unsigned long lap = sys_stopwatch_ms_stop(&_nvstore.sw);
#endif /* DEV_NVSTORE_DEBUG */

    //Now, we need to understand if we have found a valid block or not
    if (_nvstore.new_data)
    {
#if (DEV_NVSTORE_DEBUG == 1)
        iprintln(trNVSTORE, "#(%lu ms) Block: %d/%d (v%d), Wear: %s%%", lap, _nvstore.current.last_rd, NVSTORE_BLOCK_CNT, _nvstore.rd_block.hdr.version, float2str(buff, ((float)_nvstore.current.wr_cnt * 100.0f) / (float)NVSTORE_WR_CNT_MAX, 2, 16));
#endif /* DEV_NVSTORE_DEBUG */
    }
#if (DEV_NVSTORE_DEBUG == 1)
    else
    {
        iprintln(trNVSTORE, "#(%lu ms) No valid block", lap);
    }
#endif /* DEV_NVSTORE_DEBUG */

#ifdef CONSOLE_ENABLED
#if (DEV_NVSTORE_DEBUG == 1)
        console_add_menu("nvs", _nvstore_menu_items, ARRAY_SIZE(_nvstore_menu_items), "Non-Volatile Storage");
#endif /* DEV_NVSTORE_DEBUG */
#endif /* CONSOLE_ENABLED */

    return;
}

uint8_t dev_nvstore_data_size(void)
{
    return NVSTORE_BLOCK_DATA_SIZE;
}

bool dev_nvstore_read(uint8_t * data, uint8_t len)
{
    //Clear the "new data" flag
    _nvstore.new_data = false;

    if (_nvstore.current.last_rd >= NVSTORE_BLOCK_CNT)
    {
#if (DEV_NVSTORE_DEBUG == 1)
        iprintln(trNVSTORE, "#No valid block");
#endif /* DEV_NVSTORE_DEBUG */
        return false;
    }

    memcpy(data, _nvstore.rd_block.data, min(NVSTORE_BLOCK_DATA_SIZE, len));

    return true;
}

bool dev_nvstore_new_data_available(void)
{
    return _nvstore.new_data;
}

bool dev_nvstore_write(uint8_t * data, uint8_t len)
{
    nvstore_block_t _tmp_block;
    uint8_t wr_block_nr = 0;
    uint8_t calc_crc = 0;
    //uint8_t _tmp_data[NVSTORE_BLOCK_DATA_SIZE];

    if (len == 0)
    {
        // iprintln(trNVSTORE, "#Nothing to write (%d)", len);
        return false;
    }

    if (len > NVSTORE_BLOCK_DATA_SIZE)
    {
#if (DEV_NVSTORE_DEBUG == 1)
        iprintln(trNVSTORE, "#Data too big (%d > %d)", len, NVSTORE_BLOCK_DATA_SIZE);
#endif /* DEV_NVSTORE_DEBUG */
        return false;
    }

    memcpy(_tmp_block.data, data, len);
    //Fill the remainder of the data buffer with 0xFF
    if (len < NVSTORE_BLOCK_DATA_SIZE)
        memset(&_tmp_block.data[len], 0xFF, NVSTORE_BLOCK_DATA_SIZE-len);

    //Now write the EEPROM...
    wr_block_nr = _nvstore_write_next_block_data(_tmp_block.data);

    //...and then read it back to confirm it is correctly written.
    calc_crc = _nvstore_read_block((uint8_t *)&_tmp_block, wr_block_nr);
    if (calc_crc != 0)
    {
        iprintln(trNVSTORE, "#CRC ERR - block %d - 0x%02X (Exp: 0x00)", wr_block_nr, _tmp_block.crc);
        return false;
    }
    if (_tmp_block.hdr.wr_cnt != (_nvstore.current.wr_cnt + 1))
    {
        iprintln(trNVSTORE, "#WR Cnt ERR - block %d - %06lu (Exp: %06lu)", wr_block_nr, _tmp_block.hdr.wr_cnt, _nvstore.current.wr_cnt + 1);
        return false;
    }
    //Only check the length of data that was written
    if (memcmp(data, &_tmp_block.data, len) != 0)
    {
        iprintln(trNVSTORE, "#Data mismatch - block %d", wr_block_nr);
        console_print_ram(trNVSTORE, &data, 256 + wr_block_nr, len);
        console_print_ram(trNVSTORE, &_tmp_block.data, wr_block_nr, NVSTORE_BLOCK_SIZE);
        return false;
    }
    _nvstore.current.wr_cnt = _tmp_block.hdr.wr_cnt;
    _nvstore.current.last_rd = wr_block_nr;
    memcpy(&_nvstore.rd_block, &_tmp_block, NVSTORE_BLOCK_SIZE);

    // I guess if it fails we can do a couple of retries before we give up and return false
    return true;
}

#undef PRINTF_TAG
#undef EXT
/*************************** END OF FILE *************************************/
