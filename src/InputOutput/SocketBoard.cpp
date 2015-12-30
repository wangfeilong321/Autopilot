#include <SocketBoard.h>
#include <string>
#include <iterator>
#include <algorithm>

using namespace std;

SocketBoard::SocketBoard() : ifConnected(false) {
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
	doRead();
	doWrite();
}

bool SocketBoard::Connected() {
	return ifConnected;
}

bool SocketBoard::Run() {
	return Connected();
}

void SocketBoard::doRead() {
	task<UINT32>(reader->LoadAsync(sizeof(UINT32))).then([this](UINT32 size) {
		if (size < sizeof(UINT32)) {
			// The underlying socket was closed before we were able to read the whole data.
			cancel_current_task();
		}
		UINT32 dataLength = reader->ReadUInt32();
		return task<UINT32>(reader->LoadAsync(dataLength)).then([this, dataLength](UINT32 actualDataLength) {
			if (actualDataLength != dataLength) {
				// The underlying socket was closed before we were able to read the whole data.
				cancel_current_task();
			}
			//read data here
			reader->ReadDouble(); //aileron
			reader->ReadDouble(); //elevator
			reader->ReadDouble(); //rudder
			reader->ReadDouble(); //throttle
		});
	}).then([this](task<void> t) {
		try {
			// Try getting all exceptions from the continuation chain above this point.
			t.get();
			doRead();
		}
		catch (Platform::Exception^ e) {
			// Explicitly close the socket.
			ifConnected = false;
			delete socket;
		}
		catch (task_canceled&) {
			// Do not print anything here - this will usually happen because user closed the client socket.
			// Explicitly close the socket.
			ifConnected = false;
			delete socket;
		}
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

	UINT32 totalMessageSize = sizeof(UINT32) + 6 * sizeof(DOUBLE); //total message size

	task<UINT32>(writer->StoreAsync()).then([this, totalMessageSize](UINT32 writtenBytes) {
		if (writtenBytes != totalMessageSize)
			cancel_current_task();
	}).then([this](task<void> t) {
		try {
			// Try getting all exceptions from the continuation chain above this point.
			t.get();
			doWrite();
			// Everything went ok, so try to receive another string. The receive will continue until the stream is
			// broken (i.e. peer closed the socket).
		}
		catch (Platform::Exception^ e) {
			// Explicitly close the socket.
			ifConnected = false;
			delete socket;
		}
		catch (task_canceled&) {
			// Do not print anything here - this will usually happen because user closed the client socket.
			// Explicitly close the socket.
			ifConnected = false;
			delete socket;
		}
	});
}