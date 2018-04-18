#include <linux/ioctl.h>

#define ZM5304_RESET 		_IO('Z', 1)
#define ZM5304_MODE_FLASH	_IO('Z', 2)
#define ZM5304_MODE_REGULAR	_IO('Z', 3)
#define ZM5304_FLASH32		_IO('Z', 4)