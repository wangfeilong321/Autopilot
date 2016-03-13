#include <Base.h>
#include <SocketBoard.h>

using namespace std;

SocketBoard::SocketBoard(const std::shared_ptr<StateSpace>& ISS) : IState(ISS), ifConnected(false) {}

SocketBoard::~SocketBoard() {
  if (socket != nullptr) {
    delete socket;
    socket = nullptr;
  }
}

void SocketBoard::Connect() {
  socket = ref new StreamSocket();
  socket->Control->KeepAlive = true;
  socket->Control->QualityOfService = Windows::Networking::Sockets::SocketQualityOfService::LowLatency;
  reader = ref new DataReader(socket->InputStream);
  writer = ref new DataWriter(socket->OutputStream);
  String^ remoteHostAddr = ref new String(L"192.168.0.10");
  HostName^ remoteHost = ref new HostName(remoteHostAddr);
  String^ remotePort = ref new String(L"3001");
  create_task(socket->ConnectAsync(remoteHost, remotePort)).get();
  ifConnected = true;
  doWrite();
  doRead();
}

bool SocketBoard::Connected() {
  return ifConnected;
}

bool SocketBoard::Run() {
  return Connected();
}

void SocketBoard::doRead() {
  // Read first 4 bytes (length of the subsequent string).
  create_task(reader->LoadAsync(sizeof(UINT32))).then([this](unsigned int size) {
    if (size < sizeof(UINT32)) {
      // The underlying socket was closed before we were able to read the whole data.
      cancel_current_task();
    }
    unsigned int Length = reader->ReadUInt32();
    return create_task(reader->LoadAsync(Length)).then([this, Length](unsigned int actualLength) {
      if (actualLength != Length) {
        // The underlying socket was closed before we were able to read the whole data.
        cancel_current_task();
      }
      IState->setAileron(static_cast<float>(reader->ReadDouble()));
      IState->setElevator(static_cast<float>(reader->ReadDouble()));
      IState->setRudder(static_cast<float>(reader->ReadDouble()));
      IState->setThrottle(static_cast<float>(reader->ReadDouble()));
    });
  }).then([this](task<void> previousTask) {
    try {
      // Try getting all exceptions from the continuation chain above this point.
      previousTask.get();
      // Everything went ok, so try to receive another string. The receive will continue until the stream is
      // broken (i.e. peer closed the socket).
      doRead();
    }
    catch (Exception^ exception) {
      // Explicitly close the socket.
      delete socket;
      ifConnected = false;
    }
    catch (task_canceled&) {
      // Do not print anything here - this will usually happen because user closed the client socket.
      // Explicitly close the socket.
      delete socket;
      ifConnected = false;
    }
  });
}

void SocketBoard::doWrite() {
  writer->WriteUInt32(13 * sizeof(DOUBLE)); //data size in bytes
  writer->WriteDouble(IState->getRoll());  //Roll
  writer->WriteDouble(IState->getPitch()); //Pitch
  writer->WriteDouble(IState->getYaw());   //Yaw
  writer->WriteDouble(IState->getX());     // X ECEF
  writer->WriteDouble(IState->getY());     // Y ECEF
  writer->WriteDouble(IState->getZ());     // Z ECEF
  writer->WriteDouble(0.0); //vCas
  writer->WriteDouble(IState->getTemperature()); //Temperature (grad Celsius)
  writer->WriteDouble(IState->getPressure());    //Pressure (hPa)
  writer->WriteDouble(IState->getEng0Rpm());
  writer->WriteDouble(IState->getEng1Rpm());
  writer->WriteDouble(IState->getEng2Rpm());
  writer->WriteDouble(IState->getEng3Rpm());

  create_task(writer->StoreAsync()).then([this](task<unsigned int> writeTask) {
    try {
      // Try getting an exception.
      writeTask.get();
      // Everything went ok, so try to receive another string. The receive will continue until the stream is
      // broken (i.e. peer closed the socket).
      doWrite();
    }
    catch (Exception^ exception) {
      // Explicitly close the socket.
      delete socket;
      ifConnected = false;
    }
    catch (task_canceled&) {
      // Do not print anything here - this will usually happen because user closed the client socket.
      // Explicitly close the socket.
      delete socket;
      ifConnected = false;
    }
  });
}