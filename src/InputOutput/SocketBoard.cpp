#include <SocketBoard.h>
#include <string>

using namespace std;

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
	
	string dataToSend;

	dataToSend += string("s"); //start flag
	dataToSend += string("0.0,"); //timer
	dataToSend += string(to_string(38.0477f / 0.3028f) + string(",")); //altitudeASL ft
	dataToSend += string("0.0,"); //vNorth
	dataToSend += string("0.0,"); //vEast
	dataToSend += string("0.0,"); //vDown
	dataToSend += string("0.0,"); //U
	dataToSend += string("0.0,");//V
	dataToSend += string("0.0,"); //W
	dataToSend += string("0.0,"); //Roll
	dataToSend += string("0.0,"); //Pitch
	dataToSend += string("0.0,"); //Yaw
	dataToSend += string("0.0,"); //P
	dataToSend += string("0.0,"); //Q
	dataToSend += string("0.0,"); //R
	dataToSend += string("0.0,"); //velDotX
	dataToSend += string("0.0,"); //velDotY
	dataToSend += string("0.0,"); //velDotZ
	dataToSend += string("0.0,"); //vcas
	dataToSend += string("0.0,"); //Engine0 RPM
	dataToSend += string("0.0,"); //Engine1 RPM
	dataToSend += string("0.0,"); //Engine2 RPM
	dataToSend += string("0.0"); //Engine3 RPM
	dataToSend += string("f"); // finish flag
	
	auto writtenBytes = 0;
	auto bytesToWrite = dataToSend.size();
	Array<BYTE>^ buffer = ref new Array<BYTE>(bytesToWrite);
	//copy(begin(dataToSend), end(dataToSend), buffer->begin());
	for (auto i = 0; i < bytesToWrite; ++i)
		buffer[i] = dataToSend[i];
	
	writer->WriteBytes(buffer);
	
	create_task(writer->StoreAsync()).then([this, &writtenBytes](task<size_t> writeTask) {
		try {
			writtenBytes = writeTask.get();
		}
		catch (Exception^ exception) {
			String^ exceptionStr = exception->Message;
		}
	}).then([this, &bytesToWrite, &writtenBytes]() {
		if (writtenBytes == bytesToWrite) {
			bool ok = true;
		}
		else {
			bool ok = false;
		}
	});

	//auto readBytes = 0;
	create_task(reader->LoadAsync(MAX_SIZE)).then([this](task<size_t> readTask) {
		try {
			readTask.get();
		}
		catch (Exception^ exception) {
			String^ exceptionStr = exception->Message;
		}
	}).then([this]() {
		auto readBytes = MAX_SIZE;
		if (readBytes > 0) {
			string dataToRecv("",readBytes);
			Array<BYTE>^ buffer = ref new Array<BYTE>(readBytes);
			reader->ReadBytes(buffer);
			for (auto i = 0; i < readBytes; ++i)
				dataToRecv[i] = buffer[i];
		}
	});

	return true;
}