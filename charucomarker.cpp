#include "charucomarker.h"
#include <unistd.h>


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