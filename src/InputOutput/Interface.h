#ifndef INTERFACE_H
#define INTERFACE_H

class Interface {
public:
  Interface();
  virtual ~Interface();

  virtual bool Run();
  virtual bool Connected();
  virtual void Connect();
};

#endif

