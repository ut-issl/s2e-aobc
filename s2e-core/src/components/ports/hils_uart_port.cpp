/**
 * @file hils_uart_port.cpp
 * @brief Class to manage PC's COM port
 * @details Currently, this feature supports Windows Visual Studio only.(FIXME)
 * Reference: https://docs.microsoft.com/en-us/dotnet/api/system.io.ports.serialport?view=netframework-4.7.2
 * @note TODO :We need to clarify the difference with ComPortInterface
 */

#include "hils_uart_port.hpp"

// # define HILS_UART_PORT_SHOW_DEBUG_DATA

HilsUartPort::HilsUartPort(const unsigned int port_id, const unsigned int baud_rate, const unsigned int tx_buffer_size,
                           const unsigned int rx_buffer_size)
    : kPortName(GetPortName(port_id)), baud_rate_(baud_rate), kTxBufferSize(tx_buffer_size), kRxBufferSize(rx_buffer_size) {
  Initialize();
}

HilsUartPort::~HilsUartPort() {
  // Memory allocated by gcnew does not have to be explicitly deleted.
}

// Static method to convert from com port number to com port name.
std::string HilsUartPort::GetPortName(const unsigned int port_id) { return "/dev/ttyUSB" + std::to_string(port_id); }

int HilsUartPort::Initialize() {
  int ret = OpenPort();

  return ret;
}

int HilsUartPort::ClosePort() {
  close(fd_);
  fd_ = -1;
  return 0;
}

int HilsUartPort::OpenPort() {
  struct termios tio = {0};
  tio.c_cflag |= CREAD | CLOCAL | CS8;

  int i;
  int baud_rate_speed = 0;
  uint32_t available_baud_rates[] = {4800, 9600, 19200, 115200, 460800};
  speed_t baud_rate_speeds[] = {B4800, B9600, B19200, B115200, B460800};

  for (i = 0; i < sizeof(available_baud_rates) / sizeof(available_baud_rates[0]); i++) {
    if (baud_rate_ == available_baud_rates[i]) {
      baud_rate_speed = baud_rate_speeds[i];
      break;
    }
  }

  if (baud_rate_speed == 0) {
    // Invalid baud rate
    return -3;
  }

  cfsetispeed(&tio, baud_rate_speed);
  cfsetospeed(&tio, baud_rate_speed);

  cfmakeraw(&tio);

  fd_ = open(kPortName.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    // Failed to open the port
    printf("Failed to open the port: %s\n", kPortName.c_str());
    return -5;
  }

  tcsetattr(fd_, TCSANOW, &tio);
  ioctl(fd_, TCSETS, &tio);

  printf("Opened port: %s\n", kPortName.c_str());

  return 0;  // Success !!
}

int HilsUartPort::WriteTx(const unsigned char* buffer, const unsigned int offset, const unsigned int data_length) {
  if (data_length > 0) {
    printf("[%s] Writing %d bytes\n", kPortName.c_str(), data_length);
  }
  ssize_t ret = write(fd_, buffer + offset, data_length);
  //printf("[%s] Wrote %d bytes\n", kPortName.c_str(), ret);
  if (ret < 0) {
    return -1;
  }
  return 0;
}

int HilsUartPort::ReadRx(unsigned char* buffer, const unsigned int offset, const unsigned int data_length) {
  //printf("[%s] Reading %d bytes\n", kPortName.c_str(), data_length);
  ssize_t ret = read(fd_, buffer + offset, data_length);
  if (ret > 0) {
    printf("[%s] Read %d bytes\n", kPortName.c_str(), ret);
  }
  return ret;
}

int HilsUartPort::GetBytesToRead() {
  int bytes_to_read = 0;
  ioctl(fd_, FIONREAD, &bytes_to_read);
  return bytes_to_read;
}

int HilsUartPort::DiscardInBuffer() {
  tcflush(fd_, TCIFLUSH);
  return 0;
}

int HilsUartPort::DiscardOutBuffer() {
  tcflush(fd_, TCOFLUSH);
  return 0;
}
