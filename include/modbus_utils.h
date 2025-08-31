// include/modbus_utils.h

#ifndef MODBUS_UTILS_H
#define MODBUS_UTILS_H

#include <modbus.h>
#include <string>
#include <vector>
#include <mutex>

namespace test_modbus_485 {

/**
 * @brief Utility class for Modbus RTU communication using libmodbus.
 */
class ModbusUtils {
public:
    /**
     * @brief Destructor cleans up any open Modbus context.
     */
    ~ModbusUtils();

    /**
     * @brief Open and configure a Modbus RTU connection.
     * @param[out] contextReference Reference to receive the new modbus context.
     * @param[in] serialDevicePath Path to the serial port device.
     * @param[in] baudRate Communication baud rate.
     * @param[in] parityMode Parity mode: 'N'=none, 'E'=even, 'O'=odd.
     * @param[in] dataBits Number of data bits (7 or 8).
     * @param[in] stopBits Number of stop bits (1 or 2).
     * @param[in] slaveIdentifier Modbus slave identifier.
     * @return True if connection was opened successfully.
     */
    bool openRtu(modbus_t*& contextReference,
                 const std::string& serialDevicePath,
                 int baudRate = 115200,
                 char parityMode = 'N',
                 int dataBits = 8,
                 int stopBits = 1,
                 int slaveIdentifier = 1);

    /**
     * @brief Close and free the Modbus RTU context.
     * @param[in,out] contextReference Reference to the context pointer to close.
     * @return True on success.
     */
    bool closeRtu(modbus_t*& contextReference);

    /**
     * @brief Retrieve the file descriptor associated with the Modbus context.
     * @param[in] contextPointer Valid Modbus context.
     * @return File descriptor on success, -1 on error.
     */
    int getFileDescriptor(modbus_t* contextPointer);

    /**
     * @brief Reconnect using parameters from the last open call.
     * @param[in,out] contextReference Reference to the context pointer to reconnect.
     * @return True if reconnection succeeded.
     */
    bool reconnectRtu(modbus_t*& contextReference);

    int readCoils(modbus_t* contextPointer,
                  int startAddress,
                  int numberOfCoils,
                  std::vector<uint8_t>& destination);

    int readDiscreteInputs(modbus_t* contextPointer,
                           int startAddress,
                           int numberOfInputs,
                           std::vector<uint8_t>& destination);

    int readHoldingRegisters(modbus_t* contextPointer,
                             int startAddress,
                             int numberOfRegisters,
                             std::vector<uint16_t>& destination);

    int readInputRegisters(modbus_t* contextPointer,
                           int startAddress,
                           int numberOfRegisters,
                           std::vector<uint16_t>& destination);

    bool writeSingleCoil(modbus_t* contextPointer,
                         int coilAddress,
                         bool coilStatus);

    bool writeSingleRegister(modbus_t* contextPointer,
                             int registerAddress,
                             uint16_t registerValue);

    int writeMultipleCoils(modbus_t* contextPointer,
                           int startAddress,
                           const std::vector<uint8_t>& source);

    int writeMultipleRegisters(modbus_t* contextPointer,
                               int startAddress,
                               const std::vector<uint16_t>& source);

    bool maskWriteRegister(modbus_t* contextPointer,
                           int registerAddress,
                           uint16_t andMask,
                           uint16_t orMask);

    int writeAndReadRegisters(modbus_t* contextPointer,
                              int writeAddress,
                              const std::vector<uint16_t>& source,
                              int readAddress,
                              int numberOfRegisters,
                              std::vector<uint16_t>& destination);

    int reportSlaveIdentifier(modbus_t* contextPointer,
                              int maximumBytes,
                              std::vector<uint8_t>& destination);

    bool readSingleCoil(modbus_t* contextPointer,
                        int coilAddress,
                        bool& coilStatus);

    bool readSingleRegister(modbus_t* contextPointer,
                             int registerAddress,
                             uint16_t& registerValue);

private:
    bool ensureContext(modbus_t* contextPointer, const char* functionName);

    template<typename Function, typename... Arguments>
    int executeWithReconnect(modbus_t*& contextReference,
                             Function functionPointer,
                             Arguments&&... args);

    std::string lastSerialDevicePath_;
    int         lastBaudRate_;
    char        lastParityMode_;
    int         lastDataBits_;
    int         lastStopBits_;
    int         lastSlaveIdentifier_;
    std::mutex  contextMutex_;
};

template<typename Function, typename... Arguments>
int ModbusUtils::executeWithReconnect(modbus_t*& contextReference,
                                      Function functionPointer,
                                      Arguments&&... args) {
    if (!ensureContext(contextReference, __func__)) {
        return -1;
    }
    int result = functionPointer(contextReference, std::forward<Arguments>(args)...);
    if (result == -1 && reconnectRtu(contextReference)) {
        result = functionPointer(contextReference, std::forward<Arguments>(args)...);
    }
    return result;
}

} // namespace test_modbus_485

#endif // MODBUS_UTILS_H
