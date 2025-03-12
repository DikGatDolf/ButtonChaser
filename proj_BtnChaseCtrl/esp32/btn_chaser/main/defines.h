/*******************************************************************************
Global Defines
 *******************************************************************************/

//This is the firmware version of this rotator. The high nibble denotes the major
//version, while the low nibble denotes the minor version. 0x10 => V1.0
#define PROJECT_VERSION	0x10 /* Major - Minor*/



#define pinDebugRGBLED      (GPIO_NUM_8)
#define pinI2C_SDA          (GPIO_NUM_6)
#define pinI2C_SCL          (GPIO_NUM_7)



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

