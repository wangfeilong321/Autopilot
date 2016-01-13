#ifndef INTERFACE_H
#define INTERFACE_H

class Interface {
public:
  Interface();
  virtual ~Interface();

	virtual void Connect() = 0;
	virtual bool Connected() = 0;
	virtual bool Run() = 0;
};

#endif

