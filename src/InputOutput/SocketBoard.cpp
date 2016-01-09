#include <SocketBoard.h>
#include <array>

using namespace std;

SocketBoard::SocketBoard(const std::shared_ptr<StateSpace>& ISS) : IState(ISS), ifConnected(false), timer_sec(15) {}

SocketBoard::~SocketBoard() {
	if (socket != nullptr) {
		delete socket;
		socket = nullptr;
	}
}

void SocketBoard::Connect() {
	socket = ref new StreamSocket();
	socket->Control->KeepAlive = true;
	socket->Control->NoDelay = true;
	socket->Control->QualityOfService = Windows::Networking::Sockets::SocketQualityOfService::LowLatency;
	reader = ref new DataReader(socket->InputStream);
	reader->InputStreamOptions = InputStreamOptions::Partial;
	writer = ref new DataWriter(socket->OutputStream);
	String^ remoteHostAddr = ref new String(L"192.168.0.10");
	HostName^ remoteHost = ref new HostName(remoteHostAddr);
	String^ remotePort = ref new String(L"3001");
	create_task(socket->ConnectAsync(remoteHost, remotePort)).get();
	ifConnected = true;
	TimeSpan period;
	period.Duration = 1 * 10000000; // 10,000,000 ticks per second
	ThreadPoolTimer^ PeriodicTimer = ThreadPoolTimer::CreatePeriodicTimer(ref new TimerElapsedHandler([this](ThreadPoolTimer^ source) {
		timer_sec--;
		if (timer_sec <= 0)
			source->Cancel();
	}), period, ref new TimerDestroyedHandler([&](ThreadPoolTimer^ source) {}));
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
		});
	}).then([this](task<void> t) {
		try {
			// Try getting all exceptions from the continuation chain above this point.
			t.get();
			//read data from GCS here. Order is: aileron, elevator, rudder, throttle
			auto aileron = reader->ReadDouble();
			auto elevator = reader->ReadDouble();
			auto rudder = reader->ReadDouble();
			auto throttle = reader->ReadDouble();
			IState->setGCSData(aileron, elevator, rudder, throttle);
			doRead();
		}
		catch (Platform::Exception^ e) {
			// Explicitly close the socket.
			SocketErrorStatus errorStatus = SocketError::GetStatus(e->HResult);
			if (errorStatus != SocketErrorStatus::Unknown) {
				switch (errorStatus) {
					case SocketErrorStatus::HostNotFound: {
						// If hostname from user, this may indicate bad input
						// set a flag to ask user to re-enter hostname
						break;
					}
					case SocketErrorStatus::ConnectionRefused: {
						// The server might be temporarily busy
						break;
					}
					case SocketErrorStatus::NetworkIsUnreachable: {
						// Could be a connectivity issue
						break;
					}
					case SocketErrorStatus::UnreachableHost: {
						// Could be a connectivity issue
						break;
					}
					case SocketErrorStatus::NetworkIsDown: {
						// Could be a connectivity issue
						break;
					}
					default: {
						// Connection failed and no options are available
						// Try to use cached data if available 
						// may want to tell user that connect failed
						break;
					}
				}
			}
			else {
				// got an Hresult that is not mapped to an enum
				// Could be a connectivity issue
			}
			ifConnected = false;
			delete socket;
		}
		catch (task_canceled&) {
			// Do not print anything here - this will usually happen because user closed the server socket.
			// Explicitly close the socket.
			ifConnected = false;
			delete socket;
		}
	});
}

void SocketBoard::doWrite() {
	array<float, 3> Angles = IState->getAngles();
	writer->WriteUInt32(10 * sizeof(DOUBLE)); //data size in bytes
	writer->WriteDouble(timer_sec); //timer
	writer->WriteDouble(IState->getAltitude()); //altitudeASL ft
	writer->WriteDouble(Angles[0]); //Roll
	writer->WriteDouble(Angles[1]); //Pitch
	writer->WriteDouble(Angles[2]); //Yaw
	writer->WriteDouble(0.0); //vCas
	writer->WriteDouble(IState->getEng0Rpm());
	writer->WriteDouble(IState->getEng1Rpm());
	writer->WriteDouble(IState->getEng2Rpm());
	writer->WriteDouble(IState->getEng3Rpm());

	UINT32 totalMessageSize = sizeof(UINT32) + 10 * sizeof(DOUBLE); //total message size

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
			SocketErrorStatus errorStatus = SocketError::GetStatus(e->HResult);
			if (errorStatus != SocketErrorStatus::Unknown) {
				switch (errorStatus) {
					case SocketErrorStatus::HostNotFound: {
						// If hostname from user, this may indicate bad input
						// set a flag to ask user to re-enter hostname
						break;
					}
					case SocketErrorStatus::ConnectionRefused: {
						// The server might be temporarily busy
						break;
					}
					case SocketErrorStatus::NetworkIsUnreachable: {
						// Could be a connectivity issue
						break;
					}
					case SocketErrorStatus::UnreachableHost: {
						// Could be a connectivity issue
						break;
					}
					case SocketErrorStatus::NetworkIsDown: {
						// Could be a connectivity issue
						break;
					}
					default: {
						// Connection failed and no options are available
						// Try to use cached data if available 
						// may want to tell user that connect failed
						break;
					}
				}
			}
			else {
				// got an Hresult that is not mapped to an enum
				// Could be a connectivity issue
			}
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