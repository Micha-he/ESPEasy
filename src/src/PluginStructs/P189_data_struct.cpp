#include "../PluginStructs/P189_data_struct.h"

#ifdef USES_P189

// =======================================================================================================
// Constructor
// =======================================================================================================

P189_data_struct::P189_data_struct(ESPEasySerialPort portType, int8_t rxPin, int8_t txPin, uint32_t baudRate, const P189_config_t& config):_portType(portType), _rxPin(rxPin), _txPin(txPin), _baudRate(baudRate), _config(config)
{
  resetParser();
}


// =======================================================================================================
// Default configuration
// =======================================================================================================

static P189_config_t P189_defaultConfig()
{
  P189_config_t config{};

  config.version = P189_CONFIG_VERSION;
  config.minGate = P189_DEFAULT_MIN_GATE;
  config.maxGate = P189_DEFAULT_MAX_GATE;
  config.timeout = P189_DEFAULT_TIMEOUT;

  for (uint8_t gate = 0; gate < P189_NR_GATES; ++gate) {
    config.moveThreshold[gate]  = P189_FACTORY_MOVE[gate];
    config.stillThreshold[gate] = P189_FACTORY_STILL[gate];
  }
  return config;
}


// =======================================================================================================
// Load custom configuration
// =======================================================================================================

void P189_data_struct::loadConfig(taskIndex_t taskIndex, P189_config_t& config)
{
  config = P189_defaultConfig();

  P189_config_t stored{};

  LoadCustomTaskSettings(taskIndex, reinterpret_cast<uint8_t *>(&stored), sizeof(stored));

  if (stored.version != P189_CONFIG_VERSION) {
    addLog(LOG_LEVEL_INFO, F("P189: Old/invalid configuration, using defaults"));
    return;
  }

  config = stored;

  if (config.minGate > 15) {
    config.minGate = 15;
  }

  if (config.maxGate > 15) {
    config.maxGate = 15;
  }

  if (config.minGate > config.maxGate) {
    const uint32_t temp = config.minGate;
    config.minGate = config.maxGate;
    config.maxGate = temp;
  }

  if (config.timeout == 0) {
    config.timeout = P189_DEFAULT_TIMEOUT;
  }
}


// =======================================================================================================
// Save custom configuration
// =======================================================================================================

void P189_data_struct::saveConfig(taskIndex_t taskIndex, const P189_config_t& config)
{
  SaveCustomTaskSettings(taskIndex, reinterpret_cast<const uint8_t *>(&config), sizeof(config));
}


// =======================================================================================================
// Destructor
// =======================================================================================================

P189_data_struct::~P189_data_struct()
{
  disconnectSerial();
}


// =======================================================================================================
// Initialization
// =======================================================================================================

bool P189_data_struct::init()
{
  if (_serial != nullptr) {
    return true;
  }

  _serial = new (std::nothrow) ESPeasySerial(_portType, _rxPin, _txPin, false, 512);

  if (_serial == nullptr) {
    addLog(LOG_LEVEL_ERROR, F("P189: Could not allocate serial interface"));
    return false;
  }

  _serial->setRxBufferSize(512);
  _serial->begin(_baudRate);
  delay(50);
  flushSerial();
  _initialized = true;

  // Allow the LD2420 to finish booting
  delay(200);

  if (!setEnergyOutputMode()) {
    addLog(LOG_LEVEL_ERROR,F("P189: Failed to enable energy output mode"));
  } else {
    _lastModeAttemptMillis = millis();
    _lastEnergyFrameMillis = millis();
    addLog(LOG_LEVEL_INFO, F("P189: Energy output mode enabled"));
  }
  addLog(LOG_LEVEL_INFO, strformat(F("P189: LD2420 UART initialized at %d baud"), _baudRate));
  return true;
}


// =======================================================================================================
// Disconnect
// =======================================================================================================

void P189_data_struct::disconnectSerial()
{
  if (_serial != nullptr) {
    _serial->end();
    delete _serial;
    _serial = nullptr;
  }
  _initialized = false;
}


// =======================================================================================================
// Sensor processing
// =======================================================================================================

bool P189_data_struct::processSensor(EventStruct *event)
{
  if (!_initialized || _serial == nullptr) {
    return false;
  }

  bool received = false;

  while (_serial->available() > 0) {
    const int value = _serial->read();
    if (value < 0) {
      break;
    }
    if (processByte(static_cast<uint8_t>(value))) {
      received = true;
      _lastEnergyFrameMillis = millis();
    }
  }

  const uint32_t now = millis();

  // No valid energy frame received for 3 seconds, try to restore energy output mode
  if ((now - _lastEnergyFrameMillis >= 3000) && (now - _lastModeAttemptMillis >= 5000)) {
    _lastModeAttemptMillis = now;
    addLog(LOG_LEVEL_INFO, F("P189: No energy frames, restoring energy mode"));
    if (setEnergyOutputMode()) {
      _lastEnergyFrameMillis = millis();
      addLog(LOG_LEVEL_INFO, F("P189: Energy output mode restored"));
    } else {
      addLog(LOG_LEVEL_ERROR, F("P189: Failed to restore energy output mode"));
    }
  }
  return received;
}


// =======================================================================================================
// Task values
// =======================================================================================================

bool P189_data_struct::plugin_read(EventStruct *event)
{
  if (!_initialized || event == nullptr) {
    return false;
  }

  UserVar.setFloat(event->TaskIndex, 0, _presence ? 1.0f : 0.0f);
  UserVar.setFloat(event->TaskIndex, 1, static_cast<float>(_distance));
  UserVar.setFloat(event->TaskIndex, 2, static_cast<float>(_maxEnergy));
  UserVar.setFloat(event->TaskIndex, 3, static_cast<float>(_maxGate));

  return true;
}


// =======================================================================================================
// Read configuration from sensor
// =======================================================================================================

bool P189_data_struct::readConfiguration(P189_config_t& config)
{
  if (!_initialized || _serial == nullptr) {
    return false;
  }

  if (!setConfigMode(true)) {
    return false;
  }

  bool result = true;

  if (!readMinMaxTimeout(config)) {
    result = false;
  }

  if (result) {
    for (uint8_t gate = 0; gate < P189_NR_GATES; ++gate) {
      if (!readGateThreshold(gate, config.moveThreshold[gate], config.stillThreshold[gate])) {
        result = false;
        break;
      }

      delay(20);
    }
  }

  if (!setConfigMode(false)) {
    result = false;
  }

  flushSerial();

  if (result) {
    config.version = P189_CONFIG_VERSION;
    _config = config;

    addLog(LOG_LEVEL_INFO, F("P189: LD2420 configuration read successfully"));
  }

  return result;
}


// =======================================================================================================
// Apply configuration
// =======================================================================================================

bool P189_data_struct::applyConfiguration()
{
  if (!_initialized || _serial == nullptr) {
    return false;
  }

  addLog(LOG_LEVEL_INFO, strformat(F("P189: Applying LD2420 configuration, gates %d..%d, timeout=%d"), _config.minGate, _config.maxGate, _config.timeout));

  if (!setConfigMode(true)) {
    return false;
  }

  bool result = true;

  if (!setMinMaxTimeout(_config.minGate, _config.maxGate, _config.timeout)) {
    result = false;
  }

  if (result) {
    for (uint8_t gate = 0; gate < P189_NR_GATES; ++gate) {
      if (!setGateThreshold(gate)) {
        result = false;
        break;
      }
      delay(20);
    }
  }

  if (result) {
    result = setSystemMode(0x0004);
  }

  if (!setConfigMode(false)) {
    result = false;
  }

  flushSerial();

  if (result) {
    addLog(LOG_LEVEL_INFO, F("P189: LD2420 configuration applied"));
  }

  return result;
}


// =======================================================================================================
// Factory reset
// =======================================================================================================

bool P189_data_struct::factoryReset()
{
  if (!_initialized || _serial == nullptr) {
    return false;
  }

  P189_config_t factoryConfig;

  factoryConfig.version = P189_CONFIG_VERSION;
  factoryConfig.minGate = P189_DEFAULT_MIN_GATE;
  factoryConfig.maxGate = P189_DEFAULT_MAX_GATE;
  factoryConfig.timeout = P189_DEFAULT_TIMEOUT;

  for (uint8_t gate = 0; gate < P189_NR_GATES; ++gate) {
    factoryConfig.moveThreshold[gate] = P189_FACTORY_MOVE[gate];
    factoryConfig.stillThreshold[gate] = P189_FACTORY_STILL[gate];
  }

  const P189_config_t oldConfig = _config;
  _config = factoryConfig;

  addLog(LOG_LEVEL_INFO, F("P189: Writing LD2420 factory configuration"));

  if (!applyConfiguration()) {
    _config = oldConfig;
    addLog(LOG_LEVEL_ERROR, F("P189: Factory reset failed"));
    return false;
  }

  P189_config_t verifiedConfig = factoryConfig;

  if (!readConfiguration(verifiedConfig)) {
    _config = oldConfig;
    addLog(LOG_LEVEL_ERROR, F("P189: Factory reset verification failed"));
    return false;
  }

  _config = verifiedConfig;
  addLog(LOG_LEVEL_INFO, F("P189: Factory configuration restored and verified"));
  return true;
}


// =======================================================================================================
// Restart sensor
// =======================================================================================================

bool P189_data_struct::restartSensor()
{
  if (!_initialized || _serial == nullptr) {
    return false;
  }

  if (!setConfigMode(true)) {
    return false;
  }

  const bool result = sendCommand(0x0068, nullptr, 0);
  delay(250);
  flushSerial();
  return result;
}


// =======================================================================================================
// Plugin command handler
// =======================================================================================================

bool P189_data_struct::plugin_write(EventStruct *event, String& string)
{
  const String command = parseString(string, 1);

  if (!equals(command, F("ld2420"))) {
    return false;
  }

  const String parameter = parseString(string, 2);

  if (equals(parameter, F("readconfig"))) {
    P189_config_t config = _config;
    return readConfiguration(config);
  }

  if (equals(parameter, F("apply"))) {
    return applyConfiguration();
  }

  if (equals(parameter, F("factoryreset"))) {
    return factoryReset();
  }

  if (equals(parameter, F("restart"))) {
    return restartSensor();
  }

  if (equals(parameter, F("energy"))) {
    return setEnergyOutputMode();
  }

  if (equals(parameter, F("normal"))) {
    return setBasicStatusMode();
  }

  return false;
}


// =======================================================================================================
// [Task#Value]
// =======================================================================================================

bool P189_data_struct::plugin_get_config_value(EventStruct *event, String& string)
{
  const String valueName = parseString(event->String1, 1);

  if (equals(valueName, F("Presence"))) {
    string = _presence ? F("1") : F("0");
    return true;
  }

  if (equals(valueName, F("Distance"))) {
    string = String(_distance);
    return true;
  }

  if (equals(valueName, F("Energy"))) {
    string = String(_maxEnergy);
    return true;
  }

  if (equals(valueName, F("Gate"))) {
    string = String(_maxGate);
    return true;
  }

  for (uint8_t gate = 0; gate < P189_NR_GATES; ++gate) {
    const String gateName = concat(F("Gate"), gate);

    if (valueName == gateName) {
      string = String(_gateEnergy[gate]);
      return true;
    }
  }

  return false;
}


// =======================================================================================================
// Little endian helpers
// =======================================================================================================

uint16_t P189_data_struct::readLE16(const uint8_t *data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}


uint32_t P189_data_struct::readLE32(const uint8_t *data)
{
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}


void P189_data_struct::writeLE16(uint8_t *data, uint16_t value)
{
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}


void P189_data_struct::writeLE32(uint8_t *data, uint32_t value)
{
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}


// =======================================================================================================
// Serial flush
// =======================================================================================================

void P189_data_struct::flushSerial()
{
  if (_serial == nullptr) {
    return;
  }

  while (_serial->available() > 0) {
    _serial->read();
  }

  resetParser();
}


// =======================================================================================================
// Parser reset
// =======================================================================================================

void P189_data_struct::resetParser()
{
  _parseState = P189_ParseState::Header1;
  _frameLength = 0;
  _framePayloadReceived = 0;
  _frameBufferPos = 0;
}


// =======================================================================================================
// Send LD2420 command
// =======================================================================================================

bool P189_data_struct::sendCommand(uint16_t command, const uint8_t *data, uint8_t dataLength, P189_CommandResponse *response, uint32_t timeoutMs)
{
  if (_serial == nullptr) {
    return false;
  }

  uint8_t tx[80];
  uint8_t pos = 0;

  // Header
  tx[pos++] = 0xFD;
  tx[pos++] = 0xFC;
  tx[pos++] = 0xFB;
  tx[pos++] = 0xFA;

  const uint16_t length = static_cast<uint16_t>(2 + dataLength);

  writeLE16(&tx[pos], length);
  pos += 2;

  // Command
  writeLE16(&tx[pos], command);
  pos += 2;

  // Payload
  if (data != nullptr && dataLength > 0) {
    memcpy(&tx[pos], data, dataLength);
    pos += dataLength;
  }

  // Footer
  tx[pos++] = 0x04;
  tx[pos++] = 0x03;
  tx[pos++] = 0x02;
  tx[pos++] = 0x01;

  flushSerial();

  _serial->write(tx, pos);
  _serial->flush();

  // Restart: The sensor may reboot before returning an ACK.
  if (command == 0x0068) {
    return true;
  }

  uint8_t rx[96];
  uint16_t rxPos = 0;
  const uint32_t start = millis();

  while ((millis() - start) < timeoutMs) {
    while (_serial->available() > 0) {
      const int value = _serial->read();

      if (value < 0) {
        continue;
      }

      if (rxPos < sizeof(rx)) {
        rx[rxPos++] = static_cast<uint8_t>(value);
      } else {
        memmove(rx, &rx[1], sizeof(rx) - 1);
        rx[sizeof(rx) - 1] = static_cast<uint8_t>(value);
      }

      if (rxPos < 14) {
        continue;
      }

      int header = -1;

      for (uint16_t i = 0; i + 3 < rxPos; ++i) {
        if (rx[i + 0] == 0xFD && rx[i + 1] == 0xFC && rx[i + 2] == 0xFB && rx[i + 3] == 0xFA) {
          header = static_cast<int>(i);
          break;
        }
      }

      if (header < 0) {
        continue;
      }

      if (rxPos < static_cast<uint16_t>(header + 6)) {
        continue;
      }

      const uint16_t responseLength = readLE16(&rx[header + 4]);

      if (responseLength > 80) {
        continue;
      }

      const uint16_t totalLength = static_cast<uint16_t>(4 + 2 + responseLength + 4);

      if (rxPos < static_cast<uint16_t>(header + totalLength)) {
        continue;
      }

      const uint16_t footer = static_cast<uint16_t>(header + totalLength - 4);

      if (rx[footer + 0] != 0x04 || rx[footer + 1] != 0x03 || rx[footer + 2] != 0x02 || rx[footer + 3] != 0x01) {
        memmove(rx, &rx[header + 1], rxPos - header - 1);
        rxPos = rxPos - header - 1;
        continue;
      }

      const uint16_t returnedCommand = readLE16(&rx[header + 6]);
      const uint16_t expectedAck = static_cast<uint16_t>(command | 0x0100);

      if (returnedCommand != expectedAck) {
        memmove(rx, &rx[header + totalLength], rxPos - header - totalLength);
        rxPos = rxPos - header - totalLength;
        continue;
      }

      const uint16_t error = readLE16(&rx[header + 8]);

      if (response != nullptr) {
        response->command = returnedCommand;
        response->error = error;
        response->dataLength = 0;

        const uint16_t dataStart = static_cast<uint16_t>(header + 10);

        const uint16_t dataLengthReturned = responseLength >= 4 ? responseLength - 4 : 0;

        const uint8_t copyLength = static_cast<uint8_t>(min(dataLengthReturned, static_cast<uint16_t>(sizeof(response->data))));

        if (copyLength > 0) {
          memcpy(response->data, &rx[dataStart], copyLength);
        }

        response->dataLength = copyLength;
      }

      return error == 0;
    }

    delay(1);
  }

  addLog(LOG_LEVEL_ERROR, strformat(F("P189: LD2420 command timeout 0x%s"), String(command, HEX)));

  return false;
}


// =======================================================================================================
// Configuration mode
// =======================================================================================================

bool P189_data_struct::setConfigMode(bool enable)
{
  uint8_t data[2];

  if (enable) {
    writeLE16(data, 0x0002);
    return sendCommand(0x00FF, data, 2);
  }

  return sendCommand(0x00FE, nullptr, 0);
}


// =======================================================================================================
// System mode
// =======================================================================================================

bool P189_data_struct::setSystemMode(uint16_t mode)
{
  uint8_t data[6];

  writeLE16(&data[0], 0x0000);
  writeLE16(&data[2], mode);
  writeLE16(&data[4], 0x0000);

  return sendCommand(0x0012, data, sizeof(data));
}


// =======================================================================================================
// Write Min/Max/Timeout
// =======================================================================================================

bool P189_data_struct::setMinMaxTimeout(uint32_t minGate, uint32_t maxGate, uint32_t timeout)
{
  uint8_t data[18];
  uint8_t pos = 0;

  writeLE16(&data[pos], 0x0000);
  pos += 2;

  writeLE32(&data[pos], minGate);
  pos += 4;

  writeLE16(&data[pos], 0x0001);
  pos += 2;

  writeLE32(&data[pos], maxGate);
  pos += 4;

  writeLE16(&data[pos], 0x0004);
  pos += 2;

  writeLE32(&data[pos], timeout);
  pos += 4;

  return sendCommand(0x0007, data, pos);
}


// =======================================================================================================
// Read Min/Max/Timeout
// =======================================================================================================

bool P189_data_struct::readMinMaxTimeout(P189_config_t& config)
{
  uint8_t data[6];

  writeLE16(&data[0], 0x0000);
  writeLE16(&data[2], 0x0001);
  writeLE16(&data[4], 0x0004);

  P189_CommandResponse response;

  if (!sendCommand(0x0008, data, sizeof(data), &response)) {
    return false;
  }

  if (response.dataLength < 12) {
    return false;
  }

  config.minGate = readLE32(&response.data[0]);
  config.maxGate = readLE32(&response.data[4]);
  config.timeout = readLE32(&response.data[8]);

  return true;
}


// =======================================================================================================
// Write gate thresholds
// =======================================================================================================

bool P189_data_struct::setGateThreshold(uint8_t gate)
{
  if (gate >= P189_NR_GATES) {
    return false;
  }

  uint8_t data[12];
  uint8_t pos = 0;

  writeLE16(&data[pos], static_cast<uint16_t>(0x0010 + gate));
  pos += 2;

  writeLE32(&data[pos], _config.moveThreshold[gate]);
  pos += 4;

  writeLE16(&data[pos], static_cast<uint16_t>(0x0020 + gate));
  pos += 2;

  writeLE32(&data[pos], _config.stillThreshold[gate]);
  pos += 4;

  return sendCommand(0x0007, data, pos);
}


// =======================================================================================================
// Read gate thresholds
// =======================================================================================================

bool P189_data_struct::readGateThreshold(uint8_t gate, uint32_t& moving, uint32_t& still)
{
  if (gate >= P189_NR_GATES) {
    return false;
  }

  uint8_t data[4];

  writeLE16(&data[0], static_cast<uint16_t>(0x0010 + gate));
  writeLE16(&data[2], static_cast<uint16_t>(0x0020 + gate));

  P189_CommandResponse response;

  if (!sendCommand(0x0008, data, sizeof(data), &response)) {
    return false;
  }

  if (response.dataLength < 8) {
    return false;
  }

  moving = readLE32(&response.data[0]);
  still = readLE32(&response.data[4]);

  return true;
}


// =======================================================================================================
// Energy mode
// =======================================================================================================

bool P189_data_struct::setEnergyOutputMode()
{
  if (!setConfigMode(true)) {
    return false;
  }

  const bool result = setSystemMode(0x0004);

  if (!setConfigMode(false)) {
    return false;
  }

  flushSerial();

  return result;
}


// =======================================================================================================
// Normal/basic mode
// =======================================================================================================

bool P189_data_struct::setBasicStatusMode()
{
  if (!setConfigMode(true)) {
    return false;
  }

  const bool result = setSystemMode(0x0064);

  if (!setConfigMode(false)) {
    return false;
  }

  flushSerial();

  return result;
}


// =======================================================================================================
// Streaming parser
// =======================================================================================================

bool P189_data_struct::processByte(uint8_t value)
{
  switch (_parseState) {
    case P189_ParseState::Header1:
      if (value == 0xF4) {
        _parseState = P189_ParseState::Header2;
      }
      break;

    case P189_ParseState::Header2:
      if (value == 0xF3) {
        _parseState = P189_ParseState::Header3;
      } else {
        _parseState = value == 0xF4 ? P189_ParseState::Header2 : P189_ParseState::Header1;
      }
      break;

    case P189_ParseState::Header3:
      if (value == 0xF2) {
        _parseState = P189_ParseState::Header4;
      } else {
        _parseState = value == 0xF4 ? P189_ParseState::Header2 : P189_ParseState::Header1;
      }
      break;

    case P189_ParseState::Header4:
      if (value == 0xF1) {
        _frameBufferPos = 0;
        _framePayloadReceived = 0;
        _frameLength = 0;
        _parseState = P189_ParseState::Length1;
      } else {
        _parseState = value == 0xF4 ? P189_ParseState::Header2 : P189_ParseState::Header1;
      }
      break;

    case P189_ParseState::Length1:
      _frameLength = value;
      _parseState = P189_ParseState::Length2;
      break;

    case P189_ParseState::Length2:
      _frameLength |= static_cast<uint16_t>(value) << 8;

      if (_frameLength == 0 || _frameLength > sizeof(_frameBuffer)) {
        resetParser();

        if (value == 0xF4) {
          _parseState = P189_ParseState::Header2;
        }

        break;
      }

      _frameBufferPos = 0;
      _framePayloadReceived = 0;
      _parseState = P189_ParseState::Payload;
      break;

    case P189_ParseState::Payload:
      if (_framePayloadReceived < _frameLength) {
        _frameBuffer[_framePayloadReceived++] = value;

        if (_framePayloadReceived == _frameLength) {
          _parseState = P189_ParseState::Footer1;
        }
      }
      break;

    case P189_ParseState::Footer1:
      if (value == 0xF8) {
        _parseState = P189_ParseState::Footer2;
      } else {
        recoverParser(value);
      }
      break;

    case P189_ParseState::Footer2:
      if (value == 0xF7) {
        _parseState = P189_ParseState::Footer3;
      } else {
        recoverParser(value);
      }
      break;

    case P189_ParseState::Footer3:
      if (value == 0xF6) {
        _parseState = P189_ParseState::Footer4;
      } else {
        recoverParser(value);
      }
      break;

    case P189_ParseState::Footer4:
      if (value == 0xF5) {
        addLog(LOG_LEVEL_DEBUG, strformat(F("P189: RX EnergyFramePayload: %s"), formatToHex_array(_frameBuffer, _frameLength)));
        const bool result = parseEnergyFrame();
        resetParser();
        return result;
      } else {
        recoverParser(value);
      }
      break;
  }

  return false;
}


// =======================================================================================================
// Parser recovery
// =======================================================================================================

void P189_data_struct::recoverParser(uint8_t value)
{
  resetParser();

  if (value == 0xF4) {
    _parseState = P189_ParseState::Header2;
  }
}


// =======================================================================================================
// Energy frame
// =======================================================================================================

bool P189_data_struct::parseEnergyFrame()
{
  if (_frameLength != 0x23) {
    return false;
  }

  //
  // Payload:
  //
  // 0:      data type
  // 1:      presence
  // 2..3:   distance
  // 4..35:  16 gate energies
  //

  if (_frameBuffer[0] != 0x01) {
    // Some firmware revisions may report another type.
  }

  _presence = _frameBuffer[1] != 0;
  _distance = readLE16(&_frameBuffer[2]);

  _maxEnergy = 0;
  _maxGate = 0;

  for (uint8_t gate = 0; gate < P189_NR_GATES; ++gate) {
    const uint16_t offset = static_cast<uint16_t>(4 + gate * 2);

    _gateEnergy[gate] = readLE16(&_frameBuffer[offset]);

    if (_gateEnergy[gate] > _maxEnergy) {
      _maxEnergy = _gateEnergy[gate];
      _maxGate = gate;
    }
  }

  return true;
}

#endif // USES_P189