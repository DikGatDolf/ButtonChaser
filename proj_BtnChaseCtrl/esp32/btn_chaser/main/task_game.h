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
	void (*main)(void);
	void (*init)(void);
	void (*teardown)(void);
    bool (*arg_parse)(const char **arg_str_array, int arg_cnt);
} game_t;


typedef enum
{
    game_idle,            // Waiting for a game to start
    game_node_reg,        // Waiting for rollcall responses and subsequent node registration to complete
    game_init,            // Initialising the game
    game_running,         // A game is currently running
    game_paused,          // A game is paused
} game_state_e;

/******************************************************************************
Global (public) Function prototypes
******************************************************************************/

void * start_game(int index);

void end_game(void);

bool is_game_running(void);

int current_game(void);

int games_cnt(void);

const char * game_name(int index);

bool parse_game_args(int game_index, const char **arg_str_array, int arg_cnt);

#undef EXT
#endif /* __task_game_H__ */

/****************************** END OF FILE **********************************/
