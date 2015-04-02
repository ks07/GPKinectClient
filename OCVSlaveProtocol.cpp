#include <cstdio>
#include <iostream>
#include <asio.hpp>

#include "GPKinectAPI/OCVSPacketAck.h"
#include "GPKinectAPI/OCVSPacketChallenge.h"
#include "GPKinectAPI/OCVSPacketScanHeader.h"
#include "GPKinectAPI/OCVSPacketScanChunk.h"

#include "OCVSlaveProtocol.h"

// Use image file input in case a Kinect is not connected (for debugging use!)
#define FIXED_FALLBACK 1

using asio::ip::tcp;

OCVSlaveProtocol::OCVSlaveProtocol(char *host, char *port)
	: host(host)
	, port(port)
	, kinect(new KinectInterface())
	, dimgarr((uint8_t *)calloc(KinectInterface::width * KinectInterface::height, sizeof(uint8_t)))
{
	initSuccess = kinect->initKinect();
	if (initSuccess) {
		while (!kinect->getKinectDepthData(NULL, dimgarr, false)) { std::cout << '.'; } // TODO: Sleep here to throttle!
		std::cout << std::endl;
	}
}

OCVSlaveProtocol::~OCVSlaveProtocol()
{
	delete kinect;
	if (dimgarr != NULL) {
		free(dimgarr);
	}
}

bool OCVSlaveProtocol::CallDepthVision(std::vector<cv::RotatedRect> &found)
{
	bool retval = false;
	found.clear();

	if (initSuccess && kinect->getKinectDepthData(NULL, dimgarr, true))
	{
		cv::Mat input(480, 640, CV_8U, dimgarr);
		//cv::imshow("src", input);
		//cv::waitKey();

		// X and Y are inverted in the game world, so we should tranpose here
		cv::Mat transposed;
		cv::transpose(input, transposed);
		input.release();

		kinect->RunOpenCV(transposed, found);
		transposed.release();
		return true;
	}
	else if (FIXED_FALLBACK)
	{
		std::cerr << "[ERROR] Falling back to fixed image input!" << std::endl;
		cv::Mat src = cv::imread("boxbroom_painted_2.png");
		// Convert to grayscale
		cv::Mat input;
		cv::cvtColor(src, input, cv::COLOR_BGR2GRAY);
		src.release();

		// X and Y are inverted in the game world, so we should tranpose here
		cv::Mat transposed;
		cv::transpose(input, transposed);
		input.release();

		kinect->RunOpenCV(transposed, found);
		transposed.release();
		return false;
	}
	else
	{
		return false;
	}

	std::cout << "Found" << found.size() << std::endl;

	return retval;
}

/*bool OCVSlaveProtocol::CallRGBVision(std::vector<cv::RotatedRect> &found)
{
    bool retval = false;
    found.clear();

    if (initSuccess && kinect->getKinectDepthData(NULL, dimgarr, true))
    {
        cv::Mat input(480, 640, CV_8U, dimgarr);
        //cv::imshow("src", input);
        //cv::waitKey();

        // X and Y are inverted in the game world, so we should tranpose here
        cv::Mat transposed;
        cv::transpose(input, transposed);
        input.release();

        kinect->RunOpenCV(transposed, found);
        transposed.release();
        return true;
    }
    else if (FIXED_FALLBACK)
    {
        std::cerr << "[ERROR] Falling back to fixed image input!" << std::endl;
        cv::Mat src = cv::imread("boxbroom_painted_2.png");
        // Convert to grayscale
        cv::Mat input;
        cv::cvtColor(src, input, cv::COLOR_BGR2GRAY);
        src.release();

        // X and Y are inverted in the game world, so we should tranpose here
        cv::Mat transposed;
        cv::transpose(input, transposed);
        input.release();

        kinect->RunOpenCV(transposed, found);
        transposed.release();
        return false;
    }
    else
    {
        return false;
    }

    std::cout << "Found" << found.size() << std::endl;

    return retval;
}*/

void OCVSlaveProtocol::Connect()
{
	while (true)
	{
		try
		{
			std::vector<cv::RotatedRect> found;

			OCVSPacketChallenge pktChallenge;

			asio::io_service io_service;
			tcp::resolver resolver(io_service);
			tcp::resolver::query query(host, port);
			tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
			tcp::socket socket(io_service);


			asio::connect(socket, endpoint_iterator);

			for (;;)
			{
				std::vector<char> buf(128);
				asio::error_code error;

				std::cout << "Connected, sending challenge packet." << std::endl;

				pktChallenge.Pack(buf);

				socket.write_some(asio::buffer(buf), error);
				if (error == asio::error::eof)
					break; // Connection closed cleanly by peer.
				else if (error)
					throw asio::system_error(error); // Some other error.

				std::cout << "Waiting for response..." << std::endl;

				size_t len = socket.read_some(asio::buffer(buf), error);
				if (error == asio::error::eof)
					break; // Connection closed cleanly by peer.
				else if (error)
					throw asio::system_error(error); // Some other error.

				if (pktChallenge.VerifyReceived(buf)) {
					std::cout << "Good response, connected." << std::endl;

                    //HERE BE MARKERS
                    while (false /*NOTASKEDFORSCAN*/)
                    {
                        //Get and send markers
                    }

					len = socket.read_some(asio::buffer(buf), error);
					if (error == asio::error::eof)
						break; // Connection closed cleanly by peer.
					else if (error)
						throw asio::system_error(error); // Some other error.

					// TODO: Proper checking
					// Assume that if we receive a single byte it is a scan req -> so continue.
					if (len != 1) {
						// NEEDS IMPLEMENTATION
						throw asio::system_error(asio::error_code());
					}

					////////////////////////////////////////
					// Attempt to get some chunks to send //
					////////////////////////////////////////

					std::cout << "Request received, ACK'ing and responding." << std::endl;


					CallDepthVision(found);

					size_t chunk_count = found.size();

					// TODO: Get the max size from API library
					if (chunk_count > 255) {
						std::cerr << "[WARN] Found more than 255 geometries, truncating!" << std::endl;
						chunk_count = 255;
					}

					std::vector<OCVSPacket *> chunks;
					for (size_t i = 0; i < chunk_count; i++)
					{
						chunks.push_back(new OCVSPacketScanChunk(i, found.at(i)));
					}

					OCVSPacketScanHeader pktScanHead(chunks); // TODO: Empty constructor?

					// TODO: Re-arrange this, UE4 end needs to handle incomplete packets ASAP
					OCVSPacketAck::getInstance()->Pack(buf);
					socket.write_some(asio::buffer(buf), error);
					if (error == asio::error::eof)
						break; // Connection closed cleanly by peer.
					else if (error)
						throw asio::system_error(error); // Some other error.

					////////////////////////////////////////
					///////// Send the found chunks ////////
					////////////////////////////////////////

					pktScanHead.Pack(buf);
					socket.write_some(asio::buffer(buf), error);
					if (error == asio::error::eof)
						break; // Connection closed cleanly by peer.
					else if (error)
						throw asio::system_error(error); // Some other error.

					for (size_t i = 0; i < chunks.size(); i++) {
						chunks.at(i)->Pack(buf);
						socket.write_some(asio::buffer(buf), error);
						if (error == asio::error::eof)
							break; // Connection closed cleanly by peer.
						else if (error)
							throw asio::system_error(error); // Some other error.
					}

					len = socket.read_some(asio::buffer(buf), error);
					if (error == asio::error::eof)
						break; // Connection closed cleanly by peer.
					else if (error)
						throw asio::system_error(error); // Some other error.

					// TODO: Proper checking
					// Assume that if we receive a single by+te it is an ACK -> so continue.
					if (len != 1) {
						// NEEDS IMPLEMENTATION
						throw asio::system_error(asio::error_code());
					}
				}
				else {
					std::cout << "Bad response, closing." << std::endl;
					socket.shutdown(tcp::socket::shutdown_both);
					socket.close();
				}

				//std::cout.write(buf.data(), len);

			}
		}
		catch (std::exception& e)
		{
			std::cerr << e.what() << std::endl;

			// Call our disconnect method and attempt to reconnect.
		}

		// Wait some amount of time before trying again
		Sleep(5 * 1000);
	}
}