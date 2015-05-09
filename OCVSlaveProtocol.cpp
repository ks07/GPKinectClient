#include <cstdio>
#include <iostream>
#include <asio.hpp>
#include <cassert>
#include <tuple>

#include "GPKinectAPI/OCVSPacketAck.h"
#include "GPKinectAPI/OCVSPacketChallenge.h"
#include "GPKinectAPI/OCVSPacketScanHeader.h"
#include "GPKinectAPI/OCVSPacketScanChunk.h"
#include "GPKinectAPI/OCVSPacketScanReq.h"

#include "OCVSlaveProtocol.h"

using asio::ip::tcp;

OCVSlaveProtocol::OCVSlaveProtocol(char *host, char *port)
	: host(host)
	, port(port)
	, kinect(new KinectInterface())
{
	// TODO: Do we really want to be doing this here?
	// TODO: Handle init errors
	initSuccess = kinect->initKinect();
	//if (initSuccess) {
	//	while (!kinect->getKinectData(NULL, imgarr, false)) { std::cout << '.'; } // TODO: Sleep here to throttle!
	//	std::cout << std::endl;
	//}
	cv::Mat calib_src;
	kinect->GetWrappedData(calib_src, true, "floor.png");
	kinect->CalibrateDepth(calib_src);
	calib_src.release();
}

OCVSlaveProtocol::~OCVSlaveProtocol()
{
	delete kinect;
}

bool OCVSlaveProtocol::CallVision(std::vector<KinectInterface::HeightRotatedRect> &found, OCVSPacketScanReq::ScanType mode)
{
	bool retval = false;
	found.clear();

	if (mode == OCVSPacketScanReq::ScanType::SCAN_INTERACTIVE) {
		// We assume true here, the user can see the input anyway.
		retval = true;
		kinect->DebugInteractiveLoop(found); //TODO: Return found
	}
	else {
		cv::Mat input;
		retval = kinect->GetWrappedData(input, true, "closebox.png");
		kinect->ProcessDepthImage(input, found, mode == OCVSPacketScanReq::ScanType::SCAN_DEBUG);
		input.release();
	}

	std::cout << "Found" << found.size() << std::endl;

	return retval;
}

void OCVSlaveProtocol::RunCalibration()
{
	kinect->DebugCalibrationLoop();
}

void OCVSlaveProtocol::Connect()
{
	while (true)
	{
		try
		{
			std::vector<KinectInterface::HeightRotatedRect> found;

			OCVSPacketChallenge pktChallenge(0, 180);

			asio::io_service io_service;
			tcp::resolver resolver(io_service);
			tcp::resolver::query query(host, port);
			tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
			tcp::socket socket(io_service);


			asio::connect(socket, endpoint_iterator);

			for (;;)
			{
				std::vector<char> wrbuf(128);
				std::vector<char> rdbuf(128);
				asio::error_code error;

				std::cout << "Connected, sending challenge packet." << std::endl;

				pktChallenge.Pack(wrbuf);

				socket.write_some(asio::buffer(wrbuf), error);
				if (error == asio::error::eof)
					break; // Connection closed cleanly by peer.
				else if (error)
					throw asio::system_error(error); // Some other error.

				std::cout << "Waiting for response..." << std::endl;

				size_t len = socket.read_some(asio::buffer(rdbuf), error);
				if (error == asio::error::eof)
					break; // Connection closed cleanly by peer.
				else if (error)
					throw asio::system_error(error); // Some other error.

				bool haveExtraByte = false;
				char extra = 0;

				// Check if we've been given 
				if (len == OCVSPacketChallenge().GetPackedSize() + 1) {
					haveExtraByte = true;
					len--;
					extra = rdbuf[len];
				}

				if (pktChallenge.VerifyReceived(rdbuf,len)) {
					std::cout << "Good response, connected." << std::endl;

					if (haveExtraByte) {
						std::cout << "Extra byte!" << std::endl;
						len = 1;
						rdbuf[0] = extra;
						rdbuf[1] = 0; // Mark end with a null for debug

						auto would_read = socket.available();
						if (would_read > 0) {
							// This shouldn't happen...
							std::cout << "Read even more on top of extra packet!" << std::endl;
							throw asio::system_error(asio::error::no_buffer_space); // Some other error.
						}
					}
					else {
						len = socket.read_some(asio::buffer(rdbuf), error);
						if (error == asio::error::eof)
							break; // Connection closed cleanly by peer.
						else if (error)
							throw asio::system_error(error); // Some other error.
					}

					// TODO: Proper checking
					// Assume that if we receive a single byte it is a scan req -> so continue.
					if (len != 1) {
						// NEEDS IMPLEMENTATION
						throw asio::system_error(asio::error_code());
					}

					OCVSPacketScanReq scanReq(rdbuf, 0); // TODO: Check buffer behaviour...

					OCVSPacketScanReq::ScanType reqType = scanReq.GetRequestType();

					////////////////////////////////////////
					// Attempt to get some chunks to send //
					////////////////////////////////////////

					std::cout << "Request received, ACK'ing and responding." << std::endl;

					CallVision(found, reqType);

					size_t chunk_count = found.size();

					// TODO: Get the max size from API library
					if (chunk_count > 255) {
						std::cerr << "[WARN] Found more than 255 geometries, truncating!" << std::endl;
						chunk_count = 255;
					}

					std::vector<OCVSPacket *> chunks;
					for (size_t i = 0; i < chunk_count; i++)
					{
						chunks.push_back(new OCVSPacketScanChunk(i, std::get<0>(found[i]), std::get<1>(found[i]) ));
					}

					OCVSPacketScanHeader pktScanHead(chunks); // TODO: Empty constructor?

					// TODO: Re-arrange this, UE4 end needs to handle incomplete packets ASAP
					OCVSPacketAck::getInstance()->Pack(wrbuf);
					socket.write_some(asio::buffer(wrbuf), error);
					if (error == asio::error::eof)
						break; // Connection closed cleanly by peer.
					else if (error)
						throw asio::system_error(error); // Some other error.

					////////////////////////////////////////
					///////// Send the found chunks ////////
					////////////////////////////////////////

					pktScanHead.Pack(wrbuf);
					socket.write_some(asio::buffer(wrbuf), error);
					if (error == asio::error::eof)
						break; // Connection closed cleanly by peer.
					else if (error)
						throw asio::system_error(error); // Some other error.

					for (size_t i = 0; i < chunks.size(); i++) {
						chunks.at(i)->Pack(wrbuf);
						socket.write_some(asio::buffer(wrbuf), error);
						if (error == asio::error::eof)
							break; // Connection closed cleanly by peer.
						else if (error)
							throw asio::system_error(error); // Some other error.
					}

					len = socket.read_some(asio::buffer(rdbuf), error);
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