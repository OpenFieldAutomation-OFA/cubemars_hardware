#include "cubemars_hardware/can.hpp"

#include <algorithm>
#include <cstring>

#include "rclcpp/rclcpp.hpp"

namespace cubemars_hardware
{
namespace
{
constexpr const char * kLogger = "CubeMarsSystemHardware";
constexpr std::size_t kMaxCanFrames = 32;

mjbots::pi3hat::Pi3Hat::Configuration make_configuration(int can_bus)
{
  mjbots::pi3hat::Pi3Hat::CanConfiguration can_config;
  can_config.slow_bitrate = 1000000;
  can_config.fast_bitrate = 1000000;
  can_config.fdcan_frame = false;
  can_config.bitrate_switch = false;
  can_config.automatic_retransmission = true;

  mjbots::pi3hat::Pi3Hat::Configuration configuration;
  configuration.can[can_bus - 1] = can_config;
  return configuration;
}
}

bool CanSocket::connect(
  int can_bus,
  const std::vector<std::uint32_t> & /*can_ids*/,
  std::uint32_t can_mask)
{
  if (can_bus < 1 || can_bus > 5) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Invalid pi3hat CAN bus: %d", can_bus);
    return false;
  }

  can_bus_ = can_bus;
  can_mask_ = can_mask;
  rx_frames_.resize(kMaxCanFrames);
  try {
    pi3hat_ = std::make_unique<mjbots::pi3hat::Pi3Hat>(make_configuration(can_bus));
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger),
      "Could not initialize pi3hat: %s",
      exception.what());
    pi3hat_.reset();
    return false;
  }
  return true;
}

bool CanSocket::disconnect()
{
  received_frames_.clear();
  tx_frames_.clear();
  rx_frames_.clear();
  pi3hat_.reset();
  return true;
}

bool CanSocket::read_nonblocking(std::uint32_t & id, std::uint8_t data[], std::uint8_t & len)
{
  if (received_frames_.empty()) {
    return false;
  }

  const auto frame = received_frames_.front();
  received_frames_.pop_front();
  const auto copy_len = std::min<std::size_t>(frame.size, 8);
  std::memcpy(data, frame.data, copy_len);
  id = frame.id & can_mask_;
  len = static_cast<std::uint8_t>(copy_len);

  return true;
}

bool CanSocket::poll()
{
  if (!pi3hat_ || can_bus_ == 0) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "pi3hat CAN transport is not ready");
    return false;
  }

  mjbots::pi3hat::Pi3Hat::Input input;
  input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>();
  input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(
    rx_frames_.data(), rx_frames_.size());
  input.force_can_check = 1U << can_bus_;

  const auto output = pi3hat_->Cycle(input);
  if (output.error != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger),
      "pi3hat receive cycle failed with error code: %d",
      output.error);
    return false;
  }

  for (std::size_t i = 0; i < output.rx_can_size; ++i) {
    received_frames_.push_back(rx_frames_[i]);
  }
  return true;
}

bool CanSocket::write_message(std::uint32_t id, const std::uint8_t data[], std::uint8_t len)
{
  if (!pi3hat_ || can_bus_ == 0 || len > 8) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "pi3hat CAN transport is not ready");
    return false;
  }

  mjbots::pi3hat::CanFrame frame;
  frame.id = id;
  frame.bus = can_bus_;
  frame.size = len;
  frame.expect_reply = false;
  frame.expected_reply_size = 0;
  std::memcpy(frame.data, data, len);
  tx_frames_.assign(1, frame);

  mjbots::pi3hat::Pi3Hat::Input input;
  input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(
    tx_frames_.data(), tx_frames_.size());
  input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(
    rx_frames_.data(), rx_frames_.size());
  input.force_can_check = 1U << can_bus_;

  const auto output = pi3hat_->Cycle(input);
  if (output.error != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger),
      "pi3hat CAN cycle failed with error code: %d",
      output.error);
    return false;
  }

  for (std::size_t i = 0; i < output.rx_can_size; ++i) {
    received_frames_.push_back(rx_frames_[i]);
  }
  return true;
}
}