#include "_Plugin_Helper.h"
#ifdef USES_P189

// #######################################################################################################
// ########################### Plugin 189: HLK-LD2420 Radar ##############################################
// #######################################################################################################
//
// For HLK-LD2420 V1.5.4 or greater
//
// Changelog:
//
// #######################################################################################################
# include "src/PluginStructs/P189_data_struct.h"
# define PLUGIN_189
# define PLUGIN_ID_189   189
# define PLUGIN_NAME_189 "Presence - LD2420"

// =======================================================================================================
// Configuration indices
// =======================================================================================================
#define P189_BAUD_INDEX 0

// =======================================================================================================
// Plugin
// =======================================================================================================
boolean Plugin_189(uint8_t function, struct EventStruct *event, String& string)
{
  boolean success = false;
  switch (function)
  {
    // -----------------------------------------------------------------------------------------------
    // Device
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_DEVICE_ADD:
    {
      auto& dev = Device[++deviceCount];

      dev.Number           = PLUGIN_ID_189;
      dev.Type             = DEVICE_TYPE_SERIAL;
      dev.VType            = Sensor_VType::SENSOR_TYPE_QUAD;
      dev.ValueCount       = P189_NR_OUTPUT_VALUES;
      dev.OutputDataType   = Output_Data_type_t::Simple;
      dev.FormulaOption    = true;
      dev.SendDataOption   = true;
      dev.TimerOption      = true;
      dev.TimerOptional    = false;
      dev.PluginStats      = true;
      dev.ExitTaskBeforeSave = false;
      dev.CustomVTypeVar   = true;
      dev.SerialPortsAllowed = INCLUDE_NOT_CDC_SERIAL;
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Device name
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_189);
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Value names
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_189));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_189));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[2], PSTR(PLUGIN_VALUENAME3_189));
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[3], PSTR(PLUGIN_VALUENAME4_189));
      for (uint8_t i = 0; i < P189_NR_OUTPUT_VALUES; ++i) {
        ExtraTaskSettings.TaskDeviceValueDecimals[i] = 0;
      }
      // for (uint8_t i = P189_NR_OUTPUT_VALUES; i < VARS_PER_TASK; ++i) {
      //   ExtraTaskSettings.clearTaskDeviceValueName(i);
      // }
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // GPIO
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_GET_DEVICEGPIONAMES:
    {
      serialHelper_getGpioNames(event);
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Serial configuration
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_WEBFORM_SHOW_CONFIG:
    {
      string += serialHelper_getSerialTypeLabel(event);
      success = true;
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Number of values
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_GET_DEVICEVALUECOUNT:
    {
      event->Par1 = P189_NR_OUTPUT_VALUES;
      success = true;
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Sensor type
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_GET_DEVICEVTYPE:
    {
      event->sensorType = Sensor_VType::SENSOR_TYPE_QUAD;
      event->idx = -1;
      success = true;
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Defaults
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_SET_DEFAULTS:
    {
      PCONFIG(P189_BAUD_INDEX) = P189_BAUD_115200;
      success = true;
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Webform load
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_WEBFORM_LOAD:
    {
      P189_config_t config;
      P189_data_struct::loadConfig(event->TaskIndex, config);
      // ---------------------------------------------------------------------------------------------
      // Baudrate
      // ---------------------------------------------------------------------------------------------
      {
        const __FlashStringHelper *options[] = {
          F("115200"),
          F("256000")
        };
        const int optionValues[] = {
          P189_BAUD_115200,
          P189_BAUD_256000
        };
        const FormSelectorOptions selector(
          NR_ELEMENTS(optionValues),
          options,
          optionValues
        );
        selector.addFormSelector(
          F("Baud rate"),
          F("baud"),
          PCONFIG(P189_BAUD_INDEX)
        );
      }
      addFormNote(F("The LD2420 must use the same baud rate as the ESP."));
      // ---------------------------------------------------------------------------------------------
      // Detection configuration
      // ---------------------------------------------------------------------------------------------
      addFormSubHeader(F("LD2420 detection configuration"));
      addFormNumericBox(F("Minimum Gate"), F("mingate"), config.minGate, 0, P189_NR_GATES - 1);
      addFormNumericBox(F("Maximum Gate"), F("maxgate"), config.maxGate, 0, P189_NR_GATES - 1);
      addFormNumericBox(F("Timeout"), F("timeout"), config.timeout, 1, 65535);
      addUnit(F("s"));
      // ---------------------------------------------------------------------------------------------
      // Gate thresholds
      // ---------------------------------------------------------------------------------------------
      addFormSubHeader(F("Gate thresholds"));
      for (uint8_t gate = 0; gate < P189_NR_GATES; ++gate) {
        String movingLabel = F("Gate ");
        movingLabel += gate;
        movingLabel += F(" Moving");
        String movingName = F("move");
        movingName += gate;
        addFormNumericBox(movingLabel, movingName, config.moveThreshold[gate], 0, P189_MAX_THRESHOLD);
        String stillLabel = F("Gate ");
        stillLabel += gate;
        stillLabel += F(" Still");
        String stillName = F("still");
        stillName += gate;
        addFormNumericBox(stillLabel, stillName, config.stillThreshold[gate], 0, P189_MAX_THRESHOLD);
      }
      // ---------------------------------------------------------------------------------------------
      // Sensor actions
      // ---------------------------------------------------------------------------------------------
      addFormSubHeader(F("LD2420 sensor actions"));
      addFormNote(F("Use these commands from Tools > Command:"));
      addFormNote(F("ld2420,readconfig"));
      addFormNote(F("ld2420,apply"));
      addFormNote(F("ld2420,factoryreset"));
      addFormNote(F("ld2420,restart"));
      addFormNote(F("ld2420,logenergy"));
      success = true;
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Webform save
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_WEBFORM_SAVE:
    {
      const int baud = getFormItemInt(F("baud"));
      PCONFIG(P189_BAUD_INDEX) = baud == P189_BAUD_256000 ? P189_BAUD_256000 : P189_BAUD_115200;
      P189_data_struct *P189_data = static_cast<P189_data_struct *>(getPluginTaskData(event->TaskIndex));
      if (P189_data != nullptr) {
        success = P189_data->plugin_webform_save(event);
      } else {
        success = false;
        addLog(LOG_LEVEL_INFO,F("P189: WEBFORM_SAVE failed - plugin data not initialized"));
      }
      break;
    }
  
    // -----------------------------------------------------------------------------------------------
    // Initialization
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_INIT:
    {
      const int8_t rxPin = serialHelper_getRxPin(event);
      const int8_t txPin = serialHelper_getTxPin(event);
      const ESPEasySerialPort portType = serialHelper_getSerialType(event);
      const uint32_t baudRate = PCONFIG(P189_BAUD_INDEX) == P189_BAUD_256000 ? 256000 : 115200;
      P189_config_t config;
      P189_data_struct::loadConfig(event->TaskIndex, config);
      success = initPluginTaskData(event->TaskIndex, new (std::nothrow) P189_data_struct(portType, rxPin, txPin, baudRate, config));
      if (success) {
        P189_data_struct *P189_data = static_cast<P189_data_struct *>(getPluginTaskData(event->TaskIndex));
        if (P189_data != nullptr) {
          success = P189_data->init();
        }
      }
      addLog(LOG_LEVEL_INFO, strformat(F("P189: INIT, baud=%d, success=%d"), baudRate, success ? 1 : 0));
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Read
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_READ:
    {
      P189_data_struct *P189_data = static_cast<P189_data_struct *>(getPluginTaskData(event->TaskIndex));
      if (P189_data != nullptr) {
        success = P189_data->plugin_read(event);
      }
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Commands
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_WRITE:
    {
      P189_data_struct *P189_data = static_cast<P189_data_struct *>(getPluginTaskData(event->TaskIndex));
      if (P189_data != nullptr) {
        success = P189_data->plugin_write(event, string);
      }
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // [Task#Value]
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_GET_CONFIG_VALUE:
    {
      P189_data_struct *P189_data = static_cast<P189_data_struct *>(getPluginTaskData(event->TaskIndex));
      if (P189_data != nullptr) {
        success = P189_data->plugin_get_config_value(event, string);
      }
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // 10 Hz scheduler
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_TEN_PER_SECOND:
    {
      P189_data_struct *P189_data = static_cast<P189_data_struct *>(getPluginTaskData(event->TaskIndex));
      if (P189_data != nullptr) {
        success = P189_data->processSensor(event);
      }
      break;
    }

    // -----------------------------------------------------------------------------------------------
    // Exit
    // -----------------------------------------------------------------------------------------------
    case PLUGIN_EXIT:
    {
      P189_data_struct *P189_data = static_cast<P189_data_struct *>(getPluginTaskData(event->TaskIndex));
      if (P189_data != nullptr) {
        P189_data->disconnectSerial();
      }
      success = true;
      break;
    }
  }
  return success;
}
#endif // USES_P189