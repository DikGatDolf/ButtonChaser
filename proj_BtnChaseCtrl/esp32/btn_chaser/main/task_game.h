/*****************************************************************************

task_game.h

Include file for task_game.c

******************************************************************************/
#ifndef __task_game_H__
#define __task_game_H__


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

#define TASK_GAME_INTERVAL_MS      (50)     /* Task cycles at a 25Hz rate*/

/******************************************************************************
variables
******************************************************************************/
typedef struct
{
    const char *name;
	void (*cb_main)(void);
	void (*cb_init)(bool startup, bool new_game_params);
	void (*cb_teardown)(void);
    bool (*cb_arg_parse)(const char **arg_str_array, int arg_cnt, bool * new_game_params);
} game_t;

/******************************************************************************
Global (public) Function prototypes
******************************************************************************/

void * game_start(int index);

void game_end(void);

bool game_pause(void);

bool game_resume(void);

bool game_is_running(void);

bool game_is_paused(void);

int current_game(void);

int games_cnt(void);

const char * game_name(int index);

bool game_parse_args(int game_index, const char **arg_str_array, int arg_cnt);

#undef EXT
#endif /* __task_game_H__ */

/****************************** END OF FILE **********************************/
