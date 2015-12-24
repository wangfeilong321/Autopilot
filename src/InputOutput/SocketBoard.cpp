#include <SocketBoard.h>

SocketBoard::SocketBoard() : socket(ref new StreamSocket()), writer(ref new DataWriter(socket->OutputStream)), reader(ref new DataReader(socket->InputStream)), ifConnected(false) {}

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
	
	writer->WriteString(dataToSend);
	auto bytesToWrite = writer->MeasureString(dataToSend);
	auto writtenBytes = 0;

	create_task(writer->StoreAsync()).then([this, &writtenBytes](task<size_t> writeTask) {
		try {
			// Try getting an exception.
			writtenBytes = writeTask.get();
		}
		catch (Exception^ exception) {
			//rootPage->NotifyUser("Send failed with error: " + exception->Message, NotifyType::ErrorMessage);
		}
	}).then([this, &bytesToWrite, &writtenBytes]() {
		if (writtenBytes == bytesToWrite) {
			bool ok = true;
		}
		else {
			bool ok = false;
		}
	});
	
	auto readBytes = 0;
	create_task(reader->LoadAsync(MAX_SIZE)).then([this, &readBytes](task<size_t> readTask) {
		try {
			// Try getting an exception.
			readBytes = readTask.get();
		}
		catch (Exception^ exception) {
			//rootPage->NotifyUser("Send failed with error: " + exception->Message, NotifyType::ErrorMessage);
		}
	}).then([this, &readBytes]() {
		if(readBytes > 0)
			dataToRecv = reader->ReadString(readBytes);
	});

	return true;
}