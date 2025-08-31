// src/modbus_utils.cpp

#include "modbus_utils.h"
#include <cerrno>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>

bool test_modbus_485::ModbusUtils::ensureContext(modbus_t* contextPointer, const char* functionName) {
    if (!contextPointer) {
        std::cerr << "[" << functionName << "] null context\n";
        return false;
    }
    return true;
}

// Destructor
test_modbus_485::ModbusUtils::~ModbusUtils() = default;

bool test_modbus_485::ModbusUtils::openRtu(modbus_t*& contextReference,
                                           const std::string& serialDevicePath,
                                           int baudRate,
                                           char parityMode,
                                           int dataBits,
                                           int stopBits,
                                           int slaveIdentifier) {
    std::lock_guard<std::mutex> lock(contextMutex_);
    contextReference = ::modbus_new_rtu(serialDevicePath.c_str(), baudRate, parityMode, dataBits, stopBits);
    if (!contextReference) {
        std::cerr << "[openRtu] modbus_new_rtu failed\n";
        return false;
    }
    ::modbus_set_debug(contextReference, FALSE);
    if (::modbus_set_slave(contextReference, slaveIdentifier) == -1 ||
        ::modbus_connect(contextReference) == -1) {
        std::cerr << "[openRtu] set_slave or connect failed: " << modbus_strerror(errno) << "\n";
        ::modbus_free(contextReference);
        contextReference = nullptr;
        return false;
    }
    ::modbus_set_error_recovery(
        contextReference,
        static_cast<modbus_error_recovery_mode>(
            MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL));

    lastSerialDevicePath_ = serialDevicePath;
    lastBaudRate_ = baudRate;
    lastParityMode_ = parityMode;
    lastDataBits_ = dataBits;
    lastStopBits_ = stopBits;
    lastSlaveIdentifier_ = slaveIdentifier;

    int fileDescriptor = ::modbus_get_socket(contextReference);
    if (fileDescriptor < 0) {
        std::cerr << "[openRtu] invalid socket\n";
        ::modbus_free(contextReference);
        contextReference = nullptr;
        return false;
    }

    // Set timeouts (seconds, microseconds)
    uint32_t timeoutSeconds = 2;
    uint32_t timeoutMicroseconds = 0;
    ::modbus_set_response_timeout(contextReference, timeoutSeconds, timeoutMicroseconds);
    ::modbus_set_byte_timeout    (contextReference, timeoutSeconds, timeoutMicroseconds);

    termios terminalSettings;
    if (tcgetattr(fileDescriptor, &terminalSettings) != 0) {
        perror("[openRtu] tcgetattr");
        ::modbus_free(contextReference);
        contextReference = nullptr;
        return false;
    }
    cfmakeraw(&terminalSettings);

    speed_t speed;
    switch (baudRate) {
        case 9600:   speed = B9600;  break;
        case 19200:  speed = B19200; break;
        case 115200: speed = B115200;break;
        default:     speed = B115200;break;
    }
    cfsetispeed(&terminalSettings, speed);
    cfsetospeed(&terminalSettings, speed);

    terminalSettings.c_cflag = (terminalSettings.c_cflag & ~CSIZE) | (dataBits == 7 ? CS7 : CS8);
    terminalSettings.c_cflag = (stopBits == 2 ? (terminalSettings.c_cflag | CSTOPB)
                                              : (terminalSettings.c_cflag & ~CSTOPB));
    terminalSettings.c_cflag = (parityMode == 'E' ? (terminalSettings.c_cflag | PARENB)
                                                 : (terminalSettings.c_cflag & ~PARENB));
    terminalSettings.c_cflag |= (CLOCAL | CREAD);
    terminalSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
    terminalSettings.c_oflag &= ~OPOST;

    if (tcsetattr(fileDescriptor, TCSANOW, &terminalSettings) != 0) {
        perror("[openRtu] tcsetattr");
        ::modbus_free(contextReference);
        contextReference = nullptr;
        return false;
    }

    return true;
}

bool test_modbus_485::ModbusUtils::closeRtu(modbus_t*& contextReference) {
    std::lock_guard<std::mutex> lock(contextMutex_);
    if (!contextReference) {
        return true;
    }
    ::modbus_close(contextReference);
    ::modbus_free(contextReference);
    contextReference = nullptr;
    return true;
}

bool test_modbus_485::ModbusUtils::reconnectRtu(modbus_t*& contextReference) {
    std::lock_guard<std::mutex> lock(contextMutex_);
    std::cerr << "[reconnectRtu] attempting reconnect\n";
    if (contextReference) {
        ::modbus_close(contextReference);
        ::modbus_free(contextReference);
        contextReference = nullptr;
    }
    return openRtu(contextReference,
                   lastSerialDevicePath_,
                   lastBaudRate_,
                   lastParityMode_,
                   lastDataBits_,
                   lastStopBits_,
                   lastSlaveIdentifier_);
}

int test_modbus_485::ModbusUtils::getFileDescriptor(modbus_t* contextPointer) {
    return ensureContext(contextPointer, __func__)
           ? ::modbus_get_socket(contextPointer)
           : -1;
}

int test_modbus_485::ModbusUtils::readCoils(modbus_t* contextPointer,
                                            int startAddress,
                                            int numberOfCoils,
                                            std::vector<uint8_t>& destination) {
    destination.assign(numberOfCoils, 0);
    return executeWithReconnect(contextPointer,
                                ::modbus_read_bits,
                                startAddress,
                                numberOfCoils,
                                destination.data());
}

int test_modbus_485::ModbusUtils::readDiscreteInputs(modbus_t* contextPointer,
                                                     int startAddress,
                                                     int numberOfInputs,
                                                     std::vector<uint8_t>& destination) {
    destination.assign(numberOfInputs, 0);
    return executeWithReconnect(contextPointer,
                                ::modbus_read_input_bits,
                                startAddress,
                                numberOfInputs,
                                destination.data());
}

int test_modbus_485::ModbusUtils::readHoldingRegisters(modbus_t* contextPointer,
                                                       int startAddress,
                                                       int numberOfRegisters,
                                                       std::vector<uint16_t>& destination) {
    destination.assign(numberOfRegisters, 0);
    return executeWithReconnect(contextPointer,
                                ::modbus_read_registers,
                                startAddress,
                                numberOfRegisters,
                                destination.data());
}

int test_modbus_485::ModbusUtils::readInputRegisters(modbus_t* contextPointer,
                                                     int startAddress,
                                                     int numberOfRegisters,
                                                     std::vector<uint16_t>& destination) {
    destination.assign(numberOfRegisters, 0);
    return executeWithReconnect(contextPointer,
                                ::modbus_read_input_registers,
                                startAddress,
                                numberOfRegisters,
                                destination.data());
}

bool test_modbus_485::ModbusUtils::writeSingleCoil(modbus_t* contextPointer,
                                                   int coilAddress,
                                                   bool coilStatus) {
    int result = executeWithReconnect(contextPointer,
                                      ::modbus_write_bit,
                                      coilAddress,
                                      coilStatus ? 1 : 0);
    return result != -1;
}

bool test_modbus_485::ModbusUtils::writeSingleRegister(modbus_t* contextPointer,
                                                       int registerAddress,
                                                       uint16_t registerValue) {
    int result = executeWithReconnect(contextPointer,
                                      ::modbus_write_register,
                                      registerAddress,
                                      registerValue);
    return result != -1;
}

int test_modbus_485::ModbusUtils::writeMultipleCoils(modbus_t* contextPointer,
                                                     int startAddress,
                                                     const std::vector<uint8_t>& source) {
    return executeWithReconnect(contextPointer,
                                ::modbus_write_bits,
                                startAddress,
                                static_cast<int>(source.size()),
                                source.data());
}

int test_modbus_485::ModbusUtils::writeMultipleRegisters(modbus_t* contextPointer,
                                                         int startAddress,
                                                         const std::vector<uint16_t>& source) {
    return executeWithReconnect(contextPointer,
                                ::modbus_write_registers,
                                startAddress,
                                static_cast<int>(source.size()),
                                source.data());
}

bool test_modbus_485::ModbusUtils::maskWriteRegister(modbus_t* contextPointer,
                                                     int registerAddress,
                                                     uint16_t andMask,
                                                     uint16_t orMask) {
    int result = executeWithReconnect(contextPointer,
                                      ::modbus_mask_write_register,
                                      registerAddress,
                                      andMask,
                                      orMask);
    return result != -1;
}

int test_modbus_485::ModbusUtils::writeAndReadRegisters(modbus_t* contextPointer,
                                                        int writeAddress,
                                                        const std::vector<uint16_t>& source,
                                                        int readAddress,
                                                        int numberOfRegisters,
                                                        std::vector<uint16_t>& destination) {
    destination.assign(numberOfRegisters, 0);
    return executeWithReconnect(contextPointer,
                                ::modbus_write_and_read_registers,
                                writeAddress,
                                static_cast<int>(source.size()),
                                source.data(),
                                readAddress,
                                numberOfRegisters,
                                destination.data());
}

int test_modbus_485::ModbusUtils::reportSlaveIdentifier(modbus_t* contextPointer,
                                                       int maximumBytes,
                                                       std::vector<uint8_t>& destination) {
    destination.assign(maximumBytes, 0);
    return executeWithReconnect(contextPointer,
                                ::modbus_report_slave_id,
                                maximumBytes,
                                destination.data());
}

bool test_modbus_485::ModbusUtils::readSingleCoil(modbus_t* contextPointer,
                                                  int coilAddress,
                                                  bool& coilStatus) {
    std::vector<uint8_t> buffer;
    int result = readCoils(contextPointer, coilAddress, 1, buffer);
    if (result > 0) {
        coilStatus = (buffer[0] != 0);
        return true;
    }
    return false;
}

bool test_modbus_485::ModbusUtils::readSingleRegister(modbus_t* contextPointer,
                                                      int registerAddress,
                                                      uint16_t& registerValue) {
    std::vector<uint16_t> buffer;
    int result = readHoldingRegisters(contextPointer, registerAddress, 1, buffer);
    if (result > 0) {
        registerValue = buffer[0];
        return true;
    }
    return false;
}
