#pragma once
#include <memory>
#include <string>
#include <vector>

/**
 * @brief 串口奇偶校验
 *
 */
enum SerialParity {
    NO_PARITY = 0, // 无校验
    EVEN_PARITY,   // 偶校验
    ODD_PARITY     // 奇校验
};

/**
 * @brief 串口参数
 *
 */
struct SerialConfig {
    /**
   * @brief Construct a new Serial Config object
   *
   */
    SerialConfig():
        baud(115200),
        data_bits(8),
        stop_bits(1),
        parity(NO_PARITY),
        flow_control(false),
        low_latency_mode(false),
        writable(true) {}

    /**
   * @brief Construct a new Serial Config object
   *
   * @param baud
   * @param data_bits
   * @param stop_bits
   * @param parity
   * @param flow_control
   * @param low_latency_mode
   * @param writable
   */
    SerialConfig(int32_t baud, int32_t data_bits, int32_t stop_bits, SerialParity parity, bool flow_control, bool low_latency_mode, bool writable):
        baud(baud),
        data_bits(data_bits),
        stop_bits(stop_bits),
        parity(parity),
        flow_control(flow_control),
        low_latency_mode(low_latency_mode),
        writable(writable) {}

    /// @brief 波特率
    int32_t baud;
    /// @brief 数据位
    int32_t data_bits;
    /// @brief 停止位
    int32_t stop_bits;
    /// @brief 奇偶校验
    SerialParity parity;
    /// @brief 数据流控制
    bool flow_control;
    /// @brief 低延迟模式
    bool low_latency_mode;
    /// @brief 串口写入设置
    bool writable;
};

/**
 * @brief 串口读取状态
 *
 */
enum SerialResult { SUCCESS = 0,
                    TIMEOUT,
                    INTERRUPTED,
                    ERROR };

/**
 * @brief serial driver
 */
class SerialDriver {
public:
    typedef std::shared_ptr<SerialDriver> Ptr;

    /**
   * @brief Construct a new SerialDriver object
   */
    SerialDriver();

    /**
   * @brief Destructor, close serial port
   */
    ~SerialDriver();

    /**
   * @brief Open and configure the serial port.
   * @param {string} &device
   * @param {SerialConfig} config
   */
    bool Open(const std::string& device, SerialConfig config);

    /**
   * @brief Close the serial port
   */
    void Close();

    /**
   * @brief Read bytes from the serial port.
   * @param {size_t} _max_bytes
   * @param {int32_t} _timeout ms
   */
    SerialResult ReadBytes(std::vector<uint8_t>& _output, size_t _max_bytes, int32_t _timeout);

    /**
   * @brief witre byte to serial port.
   */
    int32_t Write(const std::vector<uint8_t>& _input);

    /**
   * @brief witre byte to serial port.
   * @param {string&} _input
   */
    int32_t Write(const std::string& _input);

private:
    /**
   * @brief Attempts to put serial port in low latency mode.
   */
    bool SetLowLatencyMode();

    /**
   * @brief Parses integer and enumerated baud rates into enumerated baud rates.
   * @param {int32_t} baud
   */
    int32_t ParseBaudRate(int32_t baud);

private:
    int fd_;
};