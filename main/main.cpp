#include "BTrack.h"
#include <stdio.h>
#include "mic.h"

int init()
{
	int error = i2s_install();
	
	if (error)
		return error;
	
	return 0;
}

int main()
{
	init();
	return 0;
}

extern "C"
{
	void app_main(void)
	{
		main();
	}
}
