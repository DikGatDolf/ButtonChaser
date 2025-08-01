/*******************************************************************************
Global Defines
 *******************************************************************************/

/* The "CONSOLE_ENABLED" define includes all the Console's menu interfacing and
 * such like. This also takes up about 53% of the available codespace. */
#define CONSOLE_ENABLED

/* The "MAIN_DEBUG" define includes/excludes various methods and functions which
 * only have a purpose for debugging. A decision had to be made to conserve
 * codespace and these are the guys who fell under the axe.
 * NOTE: This is only available with the Console Menu enabled */
#ifdef CONSOLE_ENABLED
	#define MAIN_DEBUG
#endif

