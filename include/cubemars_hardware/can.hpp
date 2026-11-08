#ifndef CUBEMARS_HARDWARE__CAN_HPP_
#define CUBEMARS_HARDWARE__CAN_HPP_

#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

#include "mjbots/pi3hat/pi3hat.h"

namespace cubemars_hardware
{

/**
 * @brief pi3hat CAN interface for CubeMars extended frames
 */
class CanSocket
{
public:
  /**
   * @brief Connect to CAN bus
   * @param can_bus pi3hat CAN bus number (1-5)
   * @param can_ids CAN IDs of interest
   * @param can_mask CAN IDs mask
   * @return true on success
   */
  bool connect(
    int can_bus,
    const std::vector<std::uint32_t> & can_ids,
    std::uint32_t can_mask);

  /**
   * @brief Disconnect from CAN bus
   * @return true on success
   */
  bool disconnect();

  /**
   * @brief Write message to CAN bus
   * @param id CAN extended identifier
   * @param data Data to be transmitted
   * @param len Number of bytes of data (0-8)
   * @return true on success
   */
  bool write_message(std::uint32_t id, const std::uint8_t data[], std::uint8_t len);

  /**
   * @brief Poll the CAN bus without transmitting a message
   * @return true on success
   */
  bool poll();

  /**
   * @brief Read message from CAN bus without blocking
   * @param id CAN extended identifier
   * @param data Data to be received
   * @param len Received number of bytes of data (0-8)
   * @return true on success
   */
  bool read_nonblocking(std::uint32_t & id, std::uint8_t data[], std::uint8_t & len);

private:
  std::unique_ptr<mjbots::pi3hat::Pi3Hat> pi3hat_;
  std::vector<mjbots::pi3hat::CanFrame> tx_frames_;
  std::vector<mjbots::pi3hat::CanFrame> rx_frames_;
  std::deque<mjbots::pi3hat::CanFrame> received_frames_;
  int can_bus_{0};
  std::uint32_t can_mask_;
};
}

#endif  // CUBEMARS_HARDWARE__CAN_HPP_