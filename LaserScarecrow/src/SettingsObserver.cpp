/*

   License GPL-2.0
   Part of the URI Laser Scarecrow project
   https://github.com/davidhbrown-uri/laser-scarecrow-arduino

*/

/** @see SettingsObserver.h */
#include "SettingsObserver.h"
#include <EEPROM.h>
#include "StepperController.h"
extern StepperController stepper_controller;

Settings SettingsObserver::_settings;
bool SettingsObserver::_unsaved = false;
unsigned long SettingsObserver::_lastChangedMillis = millis() - SETTINGSOBSERVER_SAVE_AFTER_MS - 1;
void SettingsObserver::init()
{
  _settings = currentSettings; // shallow copy needed
}
void SettingsObserver::process()
{
  bool foundChanges = false;

  // Check Light sensor
  if ( _settings.light_sensor_threshold != currentSettings.light_sensor_threshold)
  {
    AmbientLightSensor::applySettings(&currentSettings);
    foundChanges = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
    Serial.println(F("\r\nSettingsObserver found changes in Ambient Light Sensor settings"));
#endif
#endif
  }


  // Check Light sensor
  if ( _settings.stepper_speed_limit_percent != currentSettings.stepper_speed_limit_percent)
  {
    stepper_controller.applySettings(&currentSettings);
    foundChanges = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
    Serial.println(F("\r\nSettingsObserver found changes in Stepper speed limit"));
#endif
#endif
  }

  // Check Servo
  if ( _settings.servo_min != currentSettings.servo_min
       || _settings.servo_max != currentSettings.servo_max
       || _settings.servo_hold_time != currentSettings.servo_hold_time)
  {
    // these lines move the servo toward the min/max when settings change; useful for feat/16_bt_manual
    //only ever seeing the max. Try if/else?
    if ( _settings.servo_min != currentSettings.servo_min) {ServoController::setPulseTarget(currentSettings.servo_min);}
    else if ( _settings.servo_max != currentSettings.servo_max) {ServoController::setPulseTarget(currentSettings.servo_max);}
      ServoController::applySettings(&currentSettings);
    foundChanges = true;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
    Serial.println(F("\r\nSettingsObserver found changes in ServoController settings"));
#endif
#endif
  }

  if (foundChanges)
  {
    _unsaved = true;
    _lastChangedMillis = millis();
    _settings = currentSettings; // shallow copy needed
  }
  if (_unsaved && millis() - _lastChangedMillis > SETTINGSOBSERVER_SAVE_AFTER_MS)
  {
#ifdef DEBUG_SETTINGSOBSERVER
    bool success = SettingsObserver::save(& _settings);
#else    
    SettingsObserver::save(& _settings);
#endif
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
    Serial.print(F("\r\nSettingsObserver saving settings... "));
    Serial.println(success ? F("succeeded") : F("FAILED"));
#endif
#endif
    _unsaved = false;//might or might not be the case, but we have no way try something else
  }
}
bool SettingsObserver::save(Settings *settings_ptr)
{
  int address = 0;
  uint32_t checksum = settingsCRC(settings_ptr);
  //write type id, "LS20"
  EEPROM.update(address++, 76); // "L"
  EEPROM.update(address++, 83); // "S"
  EEPROM.update(address++, 49); // "2"
  EEPROM.update(address++, 56); // "0"
  uint16_t settingsSize = sizeof(*settings_ptr);
  EEPROM.put(address, settingsSize);
  address += 2;
  EEPROM.put(address, checksum);
  address += 4;
  EEPROM.put(address, *settings_ptr);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
  Serial.print(F("SettingsObserver::save... "));
  Serial.print(settingsSize);
  Serial.print(F(" bytes with checksum 0x"));
  Serial.println(checksum, HEX);
#endif
#endif
  return verifyStoredSettings();
}

bool SettingsObserver::checkHeader(Settings *settings_ptr)
{
  bool success = true;
  int address = 0;
  // file type id, "LS18"
  success &= EEPROM.read(address++) == 76; // "L"
  success &= EEPROM.read(address++) == 83; // "S"
  success &= EEPROM.read(address++) == 49; // "2"
  success &= EEPROM.read(address++) == 56; // "0"
  uint16_t settingsSize = 0;
  EEPROM.get(address, settingsSize);
  success &= settingsSize == sizeof(*settings_ptr);
  return success;
}
bool SettingsObserver::load(Settings *settings_ptr)
{
  Settings tmpSettings;
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
  viewEEPROM();
#endif
#endif
  bool success = true;
  int address = 0;
  uint32_t storedChecksum, liveChecksum;
  success &= checkHeader(settings_ptr);
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGS
  Serial.print("EEPROM header check ");
  Serial.println(success ? F("passed") : F("FAILED"));
#endif
#endif
  success &= verifyStoredSettings();
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGS
  Serial.print("EEPROM CRC check ");
  Serial.println(success ? F("passed") : F("FAILED"));
#endif
#endif
  if (success)
  {
    address = 6; // skip 4 byte ID, 2-byte size
    EEPROM.get(address, storedChecksum); // 4-byte CRC
    address += 4;
    EEPROM.get(address, tmpSettings);
    liveChecksum = settingsCRC(&tmpSettings);
    success &= (liveChecksum == storedChecksum);
    if (success)
    {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGS
      Serial.println("Contents of settings from EEPROM being loaded:");
      tmpSettings.printToStream(&Serial);
#endif
#endif
      *settings_ptr = tmpSettings; // shallow copy
      _settings = tmpSettings; // so we don't try to save what was just loaded! 
    }
  } // if successfully loaded the header
  return success;
}

/**
   The output of CRC32::calculate(settings_ptr, sizeof(*settings_ptr)) is NOT COMPATIBLE
   with a CRC32 recalculated on the sequence of bytes saved by EEPROM.put(*settings_ptr).
   This function does generate the same sequence of bytes and so the same checksum.

   Hmm... looking at the source code for the CRC32 library at https://github.com/bakercp/CRC32/blob/master/src/CRC32.h
   it looks like maybe for the size parameter, I should give 1, not the actual size which is
   taken from the Type parameter once it works down to calling crc.update(data,size).
   Could be it was calculating the CRCs on 22*23 bytes of unknown data after my Settings object.

   Need to get in the habit of reading other people's code when mine isn't working but I'm sure it should...

*/
uint32_t SettingsObserver::settingsCRC(Settings *settings_ptr)
{
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
  Serial.print(F("Calculating checksum of settings in RAM at 0x"));
  Serial.println((uint32_t) settings_ptr, HEX);
#endif
#endif
  byte *memory_ptr = (byte *) settings_ptr;
  CRC32 crc;
  crc.reset();
  for (int i = 0; i < (int) sizeof(*settings_ptr); i++)
  {
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
    Serial.print((byte) memory_ptr[i], HEX);
    Serial.print(' ');
#endif
#endif
    crc.update((byte) memory_ptr[i]);
  }
  uint32_t checksum = crc.finalize();
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
  Serial.print(F("Checksum = 0x"));
  Serial.println(checksum, HEX);
#endif
#endif
  return checksum;
}
bool SettingsObserver::verifyStoredSettings()
{
  int address = 0 + 4; //skip the id
  uint16_t storedSize = 0;
  EEPROM.get(address, storedSize);
  address += 2;
  uint32_t storedChecksum = 0;
  EEPROM.get(address, storedChecksum);
  address += 4;
  CRC32 crc;
  crc.reset();
  for (int i = address; i < (int) (address + storedSize); i++)
  {
    crc.update(EEPROM.read(i));
  }
  uint32_t dataChecksum = crc.finalize();
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
  Serial.println(F("Verify stored settings:"));
  Serial.print(storedSize);
  Serial.print(F(" bytes; stored CRC = "));
  Serial.println(storedChecksum, HEX);
  Serial.print(F("Recalculated data CRC = "));
  Serial.println(dataChecksum, HEX);
#endif
#endif
  return (dataChecksum == storedChecksum);
}
#ifdef DEBUG_SERIAL
#ifdef DEBUG_SETTINGSOBSERVER
void SettingsObserver::viewEEPROM()
{
  Serial.println(F("EEPROM contents:"));
  for (int row = 0; row < 64; row += 8)
  {
    Serial.print(row, HEX);
    Serial.print(F(":  "));

    for (int col = 0; col < 8; col++)
    {
      char c = EEPROM.read(row + col);
      if (c < 32 || c > 126) c = '.';
      Serial.print(c);
    }
    Serial.print(F("     hex: "));
    for (int col = 0; col < 8; col++)
    {
      Serial.print(EEPROM.read(row + col), HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}
#endif
#endif
