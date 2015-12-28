#include <SocketBoard.h>
#include <string>
#include <iterator>
#include <algorithm>

using namespace std;

SocketBoard::SocketBoard()	:	ifConnected(false) {
	socket = ref new StreamSocket();
	reader = ref new DataReader(socket->InputStream);
	reader->InputStreamOptions = InputStreamOptions::Partial;
	writer = ref new DataWriter(socket->OutputStream);
}

SocketBoard::~SocketBoard() {
	if (socket != nullptr) {
		delete socket;
		socket = nullptr;
	}
}

void SocketBoard::Connect() {
	String^ remoteHostAddr = ref new String(L"192.168.0.10");
	HostName^ remoteHost = ref new HostName(remoteHostAddr);
	String^ remotePort = ref new String(L"3001");
	create_task(socket->ConnectAsync(remoteHost, remotePort)).get();
	ifConnected = true;
}

bool SocketBoard::Connected() {
	return ifConnected;
}

bool SocketBoard::Run() {
	doWrite();
	doRead();
	return true;
}

void SocketBoard::doRead() {
	create_task(reader->LoadAsync(sizeof(UINT32))).then([this](UINT32 size) {
		if (size != sizeof(UINT32)) {
			// The underlying socket was closed before we were able to read the whole data.
			cancel_current_task();
		}
		UINT32 dataLength = reader->ReadUInt32();
		return create_task(reader->LoadAsync(dataLength)).then([this, dataLength](UINT32 actualDataLength) {
			if (actualDataLength != dataLength) {
				// The underlying socket was closed before we were able to read the whole data.
				cancel_current_task();
			}
			//read data here
			DOUBLE aileronCmd = reader->ReadDouble(); 
			DOUBLE elevatorCmd = reader->ReadDouble();
			DOUBLE rudderCmd = reader->ReadDouble();
			DOUBLE throttleCmd = reader->ReadDouble();
		});
	});
}

void SocketBoard::doWrite() {
	writer->WriteUInt32(6 * sizeof(DOUBLE)); //data size in bytes
	writer->WriteDouble(0.0); //timer
	writer->WriteDouble(38.0477f / 0.3028f); //altitudeASL ft
	writer->WriteDouble(0.8); //Roll
	writer->WriteDouble(0.7); //Pitch
	writer->WriteDouble(0.6); //Yaw
	writer->WriteDouble(0.0); //vCas

	auto totalMessageSize = sizeof(UINT32) + 6 * sizeof(DOUBLE); //total message size

	create_task(writer->StoreAsync()).then([this, totalMessageSize](size_t writtenBytes) {
		if (writtenBytes != totalMessageSize)
			cancel_current_task();
	});
}