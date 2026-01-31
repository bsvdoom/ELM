#pragma once
#include <cstdint>
#include <cstddef>

// Abstract interface for any I2C bus
class I2CInterface {
 public:
  virtual void write(uint8_t addr, uint8_t *data, size_t len) = 0;
  virtual void read(uint8_t addr, uint8_t *data, size_t len) = 0;
  virtual ~I2CInterface() = default;
};
