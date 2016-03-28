#pragma once

class Interface {
public:
  Interface() = default;
  virtual ~Interface() = default;

  virtual void Connect() = 0;
  virtual bool Connected() = 0;
  virtual bool Run() = 0;
};