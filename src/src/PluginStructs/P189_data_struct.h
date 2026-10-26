#ifndef PLUGINSTRUCTS_P189_DATA_STRUCT_H
#define PLUGINSTRUCTS_P189_DATA_STRUCT_H
#include "../../_Plugin_Helper.h"
#ifdef USES_P189

# define P189_NR_GATES              16
# define P189_NR_OUTPUT_VALUES      4
# define P189_CONFIG_VERSION        1
# define P189_BAUD_115200           0
# define P189_BAUD_256000           1
# define P189_DEFAULT_MIN_GATE      1
# define P189_DEFAULT_MAX_GATE      12
# define P189_DEFAULT_TIMEOUT       120
# define P189_MAX_TIMEOUT           65535UL
# define P189_MAX_THRESHOLD         65535UL
# define PLUGIN_VALUENAME1_189      "Presence"
# define PLUGIN_VALUENAME2_189      "Distance"
# define PLUGIN_VALUENAME3_189      "MaxEnergy"
# define PLUGIN_VALUENAME4_189      "MaxEnergyGate"

// =====================================================================================================
// LD2420 factory values
// =====================================================================================================
static constexpr uint32_t P189_FACTORY_MOVE[P189_NR_GATES] = {
  60000, 30000, 400, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250
};
static constexpr uint32_t P189_FACTORY_STILL[P189_NR_GATES] = {
  40000, 20000, 200, 200, 200, 200, 200, 150, 150, 100, 100, 100, 100, 100, 100, 100
};

// =====================================================================================================
// Configuration
// =====================================================================================================
struct P189_config_t {
  uint16_t version;
  uint32_t minGate;
  uint32_t maxGate;
  uint32_t timeout;
  uint32_t moveThreshold[P189_NR_GATES];
  uint32_t stillThreshold[P189_NR_GATES];
};

// =====================================================================================================
// LD2420 parser state
// =====================================================================================================
enum class P189_ParseState : uint8_t {Header, Length, Payload, Footer};

// =====================================================================================================
// LD2420 command frame
// =====================================================================================================
struct P189_CommandResponse {
  uint16_t command = 0;
  uint16_t error   = 0;
  uint8_t data[64] = { 0 };
  uint8_t dataLength = 0;
};

// =====================================================================================================
// Sensor abstraction
// =====================================================================================================
struct P189_data_struct : public PluginTaskData_base {
public:
  P189_data_struct(ESPEasySerialPort portType, int8_t rxPin, int8_t txPin, uint32_t baudRate, const P189_config_t& config);
  ~P189_data_struct() override;
  static void loadConfig(taskIndex_t taskIndex, P189_config_t& config);
  static void saveConfig(taskIndex_t taskIndex, const P189_config_t& config);

  // ===============================================================================================
  // Initialization
  // ===============================================================================================
  bool init();
  void disconnectSerial();

  // ===============================================================================================
  // Scheduler
  // ===============================================================================================
  bool processSensor(EventStruct *event);

  // ===============================================================================================
  // Plugin callbacks
  // ===============================================================================================
  bool plugin_read(EventStruct *event);
  bool plugin_write(EventStruct *event, String& string);
  bool plugin_get_config_value(EventStruct *event, String& string);
  bool plugin_webform_save(EventStruct *event);

  // ===============================================================================================
  // Configuration
  // ===============================================================================================
  bool readConfiguration(P189_config_t& config);
  bool applyConfiguration();
  bool factoryReset(taskIndex_t taskIndex);
  bool restartSensor();

  // ===============================================================================================
  // State
  // ===============================================================================================
  bool isRunning() const
  {
    return _initialized;
  }

private:

  // ===============================================================================================
  // Serial
  // ===============================================================================================
  ESPEasySerialPort _portType;
  int8_t _rxPin;
  int8_t _txPin;
  uint32_t _baudRate;
  ESPeasySerial *_serial = nullptr;
  bool _initialized = false;

  // ===============================================================================================
  // Configuration
  // ===============================================================================================
  P189_config_t _config;


  // ===============================================================================================
  // Sensor values
  // ===============================================================================================
  bool _presence = false;
  uint16_t _distance = 0;
  uint16_t _gateEnergy[P189_NR_GATES] = { 0 };
  uint16_t _maxEnergy = 0;
  uint8_t _maxGate = 0;

  // ===============================================================================================
  // Energy mode watchdog
  // ===============================================================================================
  uint32_t _lastEnergyFrameMillis = 0;
  uint32_t _lastModeAttemptMillis = 0;

  // ===============================================================================================
  // Streaming parser
  // ===============================================================================================
  P189_ParseState _parseState = P189_ParseState::Header;
  uint8_t _sequenceMatched = 0;
  uint8_t _frameLengthBytesReceived = 0;
  bool matchSequenceByte(uint8_t value, const uint8_t *sequence, uint8_t sequenceLength);
  uint16_t _frameLength = 0;
  uint16_t _framePayloadReceived = 0;
  uint8_t _frameBuffer[96] = { 0 };

  // ===============================================================================================
  // Parser
  // ===============================================================================================
  void resetParser();
  void flushSerial();
  bool processByte(uint8_t value);
  void recoverParser(uint8_t value);
  bool parseEnergyFrame();

  // ===============================================================================================
  // Little endian helpers
  // ===============================================================================================
  static uint16_t readLE16(const uint8_t *data);
  static uint32_t readLE32(const uint8_t *data);
  static void writeLE16(uint8_t *data, uint16_t value);
  static void writeLE32(uint8_t *data, uint32_t value);

  // ===============================================================================================
  // LD2420 command interface
  // ===============================================================================================
  bool sendCommand(uint16_t command, const uint8_t *data, uint8_t dataLength, P189_CommandResponse *response = nullptr, uint32_t timeoutMs = 1000);

  // ===============================================================================================
  // Configuration mode
  // ===============================================================================================
  bool setConfigMode(bool enable);
  bool setSystemMode(uint16_t mode);

  // ===============================================================================================
  // Min/max/timeout
  // ===============================================================================================
  bool setMinMaxTimeout(uint32_t minGate, uint32_t maxGate, uint32_t timeout);
  bool readMinMaxTimeout(P189_config_t& config);

  // ===============================================================================================
  // Gate thresholds
  // ===============================================================================================
  bool setGateThreshold(uint8_t gate);
  bool readGateThreshold(uint8_t gate, uint32_t& moving, uint32_t& still);

  // ===============================================================================================
  // Sensor modes
  // ===============================================================================================
  bool setEnergyOutputMode();
};
#endif // USES_P189
#endif // PLUGINSTRUCTS_P189_DATA_STRUCT_H