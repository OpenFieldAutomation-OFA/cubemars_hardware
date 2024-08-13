#ifndef CUBEMARS_HARDWARE__CAN_HPP_
#define CUBEMARS_HARDWARE__CAN_HPP_

#include <string>
#include <vector>
 #include <cstdint>
#include <linux/can.h>

namespace cubemars_hardware
{

/**
 * @brief SocketCAN interface for extended frame format
 */
class CanSocket
{
public:
  /**
   * @brief Connect to CAN bus
   * @param can_itf CAN interface name
   * @param can_ids CAN IDs of interest
   * @param can_mask CAN IDs mask
   * @return true on success
   */
  bool connect(std::string can_itf, const std::vector<canid_t> & can_ids, canid_t can_mask);

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
   * @brief Read message from CAN bus without blocking
   * @param id CAN extended identifier
   * @param data Data to be received
   * @param len Received number of bytes of data (0-8)
   * @return true on success
   */
  bool read_nonblocking(std::uint32_t & id, std::uint8_t data[], std::uint8_t & len);

private:
  /**
   * @brief SocketCAN socket number
   */
  int socket_;

  /**
   * @brief CAN IDs mask
   */
  std::uint32_t can_mask_;
};
}

#endif  // CUBEMARS_HARDWARE__CAN_HPP_