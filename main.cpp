#include "main.h"
#include "KinectInterface.h"
#include "OpenARScanner.h"

int main(int argc, char* argv[])
{
	OpenARScanner* scanner = new OpenARScanner();
	scanner->openARLoop();

	if (argc != 3)
	{
		std::cerr << "Usage: client <host> <port>" << std::endl;
		return 1;
	}
	printf("Here!\n");
	OCVSlaveProtocol client(argv[1], argv[2]);
	
	client.Connect();

	return 0;
}
