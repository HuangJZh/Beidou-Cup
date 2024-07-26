
#define GLOBALS_HERE
#include "includes.h"

int main(int argc, char** argv)
{
//	FILE* fp;
	int success;
//	int state;
//	int fin[2];
//	int fout[2];
//	pid_t pid;

	Parse_Arguments(argc, argv);

	success = Hardware_Init();
	if(success == -1)
	{
		Hardware_Shutdown();
		fprintf(stderr,"Hardware_Init() failed, aborting.\n");
		return(-1);
	}

	success = Pipes_Init();
	if(success == -1)
	{
		Pipes_Shutdown();
		Hardware_Shutdown();
		fprintf(stderr,"Pipes_Init() failed, aborting.\n");
		return(-1);
	}

	success = Object_Init();
	if(success == -1)
	{
		Pipes_Shutdown();
		Object_Shutdown();
		Hardware_Shutdown();
		fprintf(stderr,"Object_Init() failed, aborting.\n");
		return(-1);
	}

	success = Thread_Init();
	if(success == -1)
	{
		Thread_Shutdown();
		Pipes_Shutdown();
		Object_Shutdown();
		Hardware_Shutdown();
		fprintf(stderr,"Thread_Init() failed, aborting.\n");
		return(-1);
	}

	while(grun)
	{
		usleep(10000);
	}

	Thread_Shutdown();

	Object_Shutdown();
	Pipes_Shutdown();

	
//	Thread_Shutdown();

//	Hardware_Shutdown();
	fprintf(stdout, "All has been cleared.\n");

	return(1);

}
