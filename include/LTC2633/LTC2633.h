/*
 * Copyright 2020 ElectroOptical Innovations, LLC
 * I2C_LTC2633CTS8_AnalogOutput.h
 *
 *  Created on: Nov 5, 2019
 *      Author: simon
 */

#pragma once

//#include <AnalogOutput/AnalogOutput.h>
#include <I2CBus/I2CInterface.h>
#include <cstdint>

//  fixme how to break this up into 2 seperate analog outputs -> have it own the
//  analog outputs as members and give out references
namespace LTC2633 {
enum class CA0Mode : uint8_t { kLow = 0x0, kFloating = 0x1, kHigh = 0x2 };
enum class Command : uint8_t {
  kWriteRegisterN = 0x00,
  kUpdateDacRegisterN = 0x10,
  kWriteToInputRegisterN_UpdateAll = 0x20,
  kWriteToAndUpdateDacRegisterN = 0x30,
  kPowerDownN = 0x40,
  kPowerDownChip = 0x50,
  kSelectInternalReference = 0x60,
  kSelectExternalReference = 0x70,
  kIllegal0 = 0x80,
  kIllegal1 = 0xa0,
  kIllegal2 = 0xb0,
  kIllegal3 = 0xc0,
  kIllegal4 = 0xd0,
  kIllegal5 = 0xe0,
  kNoOp = 0xf0
};

static const double constexpr kVoltageReferenceNominal = 2.5;
static const uint32_t kDataBits = 16;
enum class Channel : uint8_t { kCHA = 0x0, kCHB = 0x1, kAll = 0xf };
static const constexpr uint8_t kSlaveAddressGlobal = 0b1110011;
static const constexpr uint8_t kSlaveAddressBase = 0b0010000;

/*
 * Upper bits followed by lower bits,
 * */
template<typename Device_t, size_t kBits>
inline void SendCommand(Device_t* p_device, CA0Mode ca0, Command cmd, Channel channel, uint16_t data) {
  const uint32_t slave_address = static_cast<uint8_t>(kSlaveAddressBase +
                                           static_cast<uint8_t>(ca0));
  const uint16_t shifted_data = data << (kDataBits - kBits);
  const uint8_t command_byte =
      static_cast<uint8_t>(cmd) | static_cast<uint8_t>(channel);
  p_device->InsertOperation(
      {I2C::OperationType::kWrite, I2C::MakeSlaveWriteAddress(slave_address)});
  p_device->InsertOperation({I2C::OperationType::kStart});
  p_device->InsertOperation({I2C::OperationType::kWrite, command_byte});
  p_device->InsertOperation({I2C::OperationType::kContinue});
  p_device->InsertOperation({I2C::OperationType::kWrite,
                   static_cast<uint8_t>(0xff & (shifted_data >> 8))});
  p_device->InsertOperation({I2C::OperationType::kContinue});
  p_device->InsertOperation(
      {I2C::OperationType::kWrite, static_cast<uint8_t>(0xff & shifted_data)});
  p_device->InsertOperation({I2C::OperationType::kContinue});
}

template<typename Device_t, size_t kBits>
inline void Write(Device_t* p_device, Channel channel, uint16_t value) {
  SendCommand<Device_t, kBits>(p_device, Command::kWriteToAndUpdateDacRegisterN, channel, value);
}

#if 0
template<typename AnalogOutput_t>
class I2C_AnalogOutput final : public AnalogOutput_t {
 private:
  uint32_t last_setting_ = 0;
  bool update_pending_ = false;

 public:
  virtual void Update(void) { update_pending_ = true; }

  virtual void Setup(void) { Reset(); }
  virtual void Reset(void) {
    update_pending_ = false;
    last_setting_ = 0;
  }
  virtual uint32_t HardwareRead(void) const { return get_setting(); }
  bool UpdateIsPending(void) const { return update_pending_; }
  uint16_t GetPendingValue(void) {
    uint16_t out = static_cast<uint16_t>(get_setting());

    update_pending_ = false;
    last_setting_ = out;
    return out;
  }

  explicit I2C_AnalogOutput(const uint32_t bits)
      : AnalogOutput_t{bits} {}
  virtual ~I2C_AnalogOutput(void) {}
};


template<size_t kBits>
class Driver final : public I2CDeviceBase {
 private:
  enum class State {
    kReset,
    kOperating,
  };

  State state_ = State::kReset;
  const uint8_t kSlaveAddress;

 private:
  I2C_AnalogOutput channel_a_analogout_{kBits};
  I2C_AnalogOutput channel_b_analogout_{kBits};
  Channel active_channel_ = Channel::kCHA;
  void SetNextActiveChannel(void) {
    switch (active_channel_) {
    case (Channel::kAll):
      active_channel_ = Channel::kCHA;
      break;
    case (Channel::kCHA):
      active_channel_ = Channel::kCHB;
      break;
    case (Channel::kCHB):
      active_channel_ = Channel::kCHA;
      break;
    default:
      assert(0);
    }
  }
  I2C_AnalogOutput &GetActiveChannelAnalogOutput(void) {
    switch (active_channel_) {
    case (Channel::kAll):
      return channel_a_analogout_;
    case (Channel::kCHA):
      return channel_a_analogout_;
    case (Channel::kCHB):
      return channel_b_analogout_;
    default:
      assert(0);
      return channel_a_analogout_;
    }
  }

 public:
  /*
   * There's no read
   * Start -> write Address -> Command << 4 | Address -> 2xdata
   * The command is
   *
   * */
  I2C_AnalogOutput &GetChannelA_AnalogOut(void) {
    return channel_a_analogout_;
  }
  I2C_AnalogOutput &GetChannelB_AnalogOut(void) {
    return channel_b_analogout_;
  }

  virtual void Reset(void) {
    channel_a_analogout_.Reset();
    channel_b_analogout_.Reset();
    state_ = State::kReset;
  }

  virtual void PushData(uint8_t) {
    assert(0); //  this device has no read
  }

  virtual void Run(void) {
    switch (state_) {
    case (State::kReset): {
      SendCommand(Command::kSelectInternalReference, Channel::kAll, 0);
      state_ = State::kOperating;
      break;
    }
    case (State::kOperating): {
      I2C_AnalogOutput &channel_aout =
          GetActiveChannelAnalogOutput();
      if (channel_aout.UpdateIsPending()) {
        Write(active_channel_, channel_aout.GetPendingValue());
      }
      SetNextActiveChannel();
      break;
    }
    default:
      assert(0);
    }
  }

  explicit Driver(CA0Mode ca0)
      : kSlaveAddress{static_cast<uint8_t>(kSlaveAddressBase +
                                           static_cast<uint8_t>(ca0))} {}
  virtual ~Driver(void) {}
};
#endif
}  //  namespace LTC2633
