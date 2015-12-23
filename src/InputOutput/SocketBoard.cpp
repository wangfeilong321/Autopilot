#include <SocketBoard.h>

SocketBoard::SocketBoard() : socket(ref new StreamSocket()), ifConnected(false) {}

SocketBoard::~SocketBoard() {
	if (socket != nullptr) {
		delete socket;
		socket = nullptr;
	}
}

void SocketBoard::Connect() {
	String^ remotehostaddr = ref new String(L"192.168.0.10");
	HostName^ remotehost = ref new HostName(remotehostaddr);
	String^ remoteport = ref new String(L"3001");
	create_task(socket->ConnectAsync(remotehost, remoteport)).get();
	ifConnected = true;
}

bool SocketBoard::Connected() {
	return ifConnected;
}

bool SocketBoard::Run() {
	create_task(socket->InputStream->ReadAsync(buffer, 1024, InputStreamOptions::None)).then([this](IBuffer^ /*buf*/) {
	});

	create_task(socket->OutputStream->WriteAsync(buffer)).then([this](uint32 /*writtenBytes*/) {
	});
	return true;
}