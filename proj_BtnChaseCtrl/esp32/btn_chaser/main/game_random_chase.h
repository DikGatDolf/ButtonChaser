/*****************************************************************************

game_random_chase.h

Include file for game_random_chase.c

******************************************************************************/
#ifndef __game_random_chase_H__
#define __game_random_chase_H__


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

void game_random_chase_main(void);
void game_random_chase_init(bool startup, bool new_game_params);
void game_random_chase_teardown(void);
bool game_random_chase_arg_parser(const char **arg_str_array, int arg_cnt, bool * new_game_params);

#undef EXT
#endif /* __game_random_chase_H__ */

/****************************** END OF FILE **********************************/
