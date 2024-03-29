#include "main.h"
#include "KinectInterface.h"

int main(int argc, char* argv[])
{
	if (argc != 3)
	{
		std::cerr << "Usage: client <host> <port>" << std::endl;
		return 1;
	}

	OCVSlaveProtocol client(argv[1], argv[2]);
	client.RunCalibration();
	client.Connect();

	return 0;
}
