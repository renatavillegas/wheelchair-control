#include "charucomarker.h"
#include <unistd.h>

// Display some info to user
void CharucoMarker::show_info()
{
	cout << "Each door have an unique CharucoMarker "
	<<"that will be used to define from position from the chair.";
}

void CharucoMarker::hello_thread()
{
	while(true)
	{
		cout << "hello thread!\n";
		sleep(10);
	}
}

void CharucoMarker::start_thread()
{
	thread mt1(&CharucoMarker::hello_thread, this);
	mt1.detach();
}