#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Interface.h>
#include <SensorBoard.h>
#include <EngineBoard.h>
#include <SocketBoard.h>
#include <StateSpace.h>

#include <memory>
#include <string>

class Controller : public Interface {
public:
	Controller();
	virtual ~Controller() = default;

	virtual void Connect();
	virtual bool Connected();
	virtual bool Run();

protected:
	class wexception {
	public:
		explicit wexception(const std::wstring &msg) : msg_(msg) {}
		virtual const wchar_t *wwhat() const { return msg_.c_str(); }
	private:
		std::wstring msg_;
	};

private:
	std::shared_ptr<StateSpace> IState;
	std::shared_ptr<SensorBoard> ISensor;
	std::shared_ptr<EngineBoard> IEngine;
	std::shared_ptr<SocketBoard> ISocket;
};

#endif