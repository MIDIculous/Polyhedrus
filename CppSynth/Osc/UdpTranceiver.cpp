#include "UdpTranceiver.h"

UdpTranceiver::UdpTranceiver(int receivePort, int sendPort)
	: UdpTranceiver(receivePort, sendPort, "127.0.0.1")
{

}

UdpTranceiver::UdpTranceiver(int receivePort, int sendPort, std::string sendIp)
{
	// ----- Setup Sending -----

	udp::resolver resolver(ioServiceSend);
	udp::resolver::query query(udp::v4(), sendIp, std::to_string(sendPort));
	sendEndpoint = *resolver.resolve(query);

	// Open a random socket for sending
	sendSocket = new udp::socket(ioServiceSend);
	sendSocket->open(udp::v4());

	// ------ Setup Receiving -------

	receiveSocket = new udp::socket(ioServiceReceive, udp::endpoint(udp::v4(), receivePort));
}

UdpTranceiver::~UdpTranceiver()
{
	delete sendSocket;
	delete receiveSocket;
}

std::vector<uint8_t> UdpTranceiver::Receive()
{
	udp::endpoint remote_endpoint;
	boost::system::error_code error;
	int byteCount = receiveSocket->receive_from(boost::asio::buffer(recveiveBuffer), remote_endpoint, 0, error);
	
	std::vector<uint8_t> output;
	
	for (size_t i = 0; i < byteCount; i++)
		output.push_back(recveiveBuffer[i]);
		
	return output;
}

void UdpTranceiver::Send(std::vector<uint8_t> message)
{
	sendSocket->send_to(boost::asio::buffer(message), sendEndpoint);
}
