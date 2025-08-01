/*****************************************************************************

game_demo.h

Include file for game_demo.c

******************************************************************************/
#ifndef __game_demo_H__
#define __game_demo_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"

/******************************************************************************
Macros
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
Global (public) Function prototypes
******************************************************************************/

void game_demo_main(void);
void game_demo_init(bool startup, bool new_game_params);
void game_demo_teardown(void);
bool game_demo_arg_parser(const char **arg_str_array, int arg_cnt, bool * new_game_params);

#undef EXT
#endif /* __game_demo_H__ */

/****************************** END OF FILE **********************************/
