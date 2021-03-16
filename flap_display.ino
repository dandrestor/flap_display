#include <Stepper.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>

#define ENABLE_DEBUG

/********** RESET ***************/

/*
 * Pentru a reseta unitatea la configuratia default:
 *  - pune PIN 2 la ground si reseteaza Arduino-ul (buton sau power on/off)
 *  - dupa ce configuratia se reseteaza, LED-ul de pe Arduino incepe sa clipeasca.
 *  - pune PIN 2 floating (neconectat) si reseteaza Arduino-ul din nou
 *
 *  Configuratia default:
 *  - MAC address: DE:AD:BE:EF:FE:ED
 *  - IP address: 192.168.5.39
 *  - stepsPerRevolution: 2048
 *  - stepsPerFlap: 38
 *  - flapsPerRevolution: 54
 *  - stepperSpeed: 10
 *  - numDevices: 0
 */

/********************************/

#ifdef ENABLE_DEBUG
#define DebugSay(...) _DebugSay(__VA_ARGS__)
#else
#define DEBUG(X) (void *)0
#endif

struct DeviceConfig
{
  int stepperPins[4];
  int hallPin;
  int hallOffset;
};

Stepper **steppers = NULL;
int *currentFlap = NULL;
int *targetFlap = NULL;
int *pendingSteps = NULL;
int *lastHallReading = NULL;

struct scd {
  byte mac[6];
  byte ip[4];
  int stepsPerRevolution;
  int stepsPerFlap;
  int flapsPerRevolution;
  int stepperSpeed;
  int numDevices;
};
scd config;

DeviceConfig *devices = NULL;

EthernetServer configServer(23);
bool telnetClientConnected = false;
String currentInput = "";

const byte maxLineLength = 80;


void TelnetSay(const char *format...)
{
  static char buf[maxLineLength];

  va_list args;
  va_start(args, format);
  vsnprintf(buf, maxLineLength, format, args);
  va_end(args);

  configServer.write(buf);
}

void TelnetSay(const __FlashStringHelper *format...)
{
  static char formatBuf[maxLineLength];
  static char lineBuf[maxLineLength];

  PGM_P p = reinterpret_cast<PGM_P>(format);
  size_t n = 0;
  while (n < maxLineLength - 1)
  {
    formatBuf[n++] = pgm_read_byte(p++);
  }
  formatBuf[n] = 0;

  va_list args;
  va_start(args, format);
  vsnprintf(lineBuf, maxLineLength, formatBuf, args);
  va_end(args);

  configServer.write(lineBuf);
}

void _DebugSay(const __FlashStringHelper *format...)
{
  static char formatBuf[maxLineLength];
  static char lineBuf[maxLineLength];
  static char lastLineBuf[maxLineLength] = { 0 };
  static bool repetitions = false;

  PGM_P p = reinterpret_cast<PGM_P>(format);
  size_t n = 0;
  while (n < maxLineLength - 1)
  {
    formatBuf[n++] = pgm_read_byte(p++);
  }
  formatBuf[n] = 0;

  va_list args;
  va_start(args, format);
  vsnprintf(lineBuf, maxLineLength, formatBuf, args);
  va_end(args);

  if (strcmp(lineBuf, lastLineBuf) == 0)
  {
    repetitions = true;
    return;
  }

  if (repetitions)
  {
    Serial.println(F("Previous message was repeated multiple times."));
    repetitions = false;
  }
  memcpy(lastLineBuf, lineBuf, maxLineLength);

  Serial.print(lineBuf);
}

void FactoryReset()
{
  static const scd defaults = {
    { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED },
    { 192, 168, 5, 39 },
    2048, 38, 54, 10, 0
  };
  EEPROM.put(0, defaults);
}

void ReallocateDeviceArrays()
{
  free(devices);
  free(steppers);
  free(currentFlap);
  free(targetFlap);
  free(pendingSteps);
  free(lastHallReading);

  size_t size = sizeof(*devices) * config.numDevices;
  devices = (DeviceConfig *)malloc(size);
  memset(devices, 0, size);

  size = sizeof(*steppers) * config.numDevices;
  steppers = (Stepper **)malloc(size);
  memset(steppers, 0, size);

  size = sizeof(int) * config.numDevices;
  currentFlap = (int *)malloc(size);
  memset(currentFlap, 0, size);

  size = sizeof(int) * config.numDevices;
  targetFlap = (int *)malloc(size);
  memset(targetFlap, 0, size);

  size = sizeof(int) * config.numDevices;
  pendingSteps = (int *)malloc(size);
  memset(pendingSteps, 0, size);

  size = sizeof(int) * config.numDevices;
  lastHallReading = (int *)malloc(size);
  memset(lastHallReading, 0, size);
}

void ReadConfiguration()
{
  EEPROM.get(0, config);
  ReallocateDeviceArrays();
  for (int i = 0; i < config.numDevices; ++i)
  {
    size_t offset = sizeof(config) + i * sizeof(*devices);
    EEPROM.get(offset, devices[i]);
  }
}

void WriteConfiguration()
{
  EEPROM.put(0, config);
  for (int i = 0; i < config.numDevices; ++i)
  {
    size_t offset = sizeof(config) + i * sizeof(*devices);
    EEPROM.put(offset, devices[i]);
  }

}

void InitializeSteppers()
{
  for (int i = 0; i < config.numDevices; ++i)
  {

    Stepper *s = new Stepper(config.stepsPerRevolution,
                             devices[i].stepperPins[0],
                             devices[i].stepperPins[2],  // sic
                             devices[i].stepperPins[1],  // sic
                             devices[i].stepperPins[3]);
    steppers[i] = s;
    pinMode(devices[i].hallPin, INPUT);
    steppers[i]->setSpeed(config.stepperSpeed);

    // set parameters to reset to hall sensor
    pendingSteps[i] = -1;
    currentFlap[i] = config.flapsPerRevolution - 1;
    targetFlap[i] = 0;
    lastHallReading[i] = HIGH;
  }

  // perform hall reset
  bool allSteppersReset = false;
  while (!allSteppersReset)
  {
    allSteppersReset = true;
    for (int d = 0; d < config.numDevices; ++d)
    {
      RunDevice(d);
      if (pendingSteps[d] != 0)
      {
        allSteppersReset = false;
      }
    }
  }
}

void TurnOffMotor(int deviceId)
{
  digitalWrite(devices[deviceId].stepperPins[0], LOW);
  digitalWrite(devices[deviceId].stepperPins[1], LOW);
  digitalWrite(devices[deviceId].stepperPins[2], LOW);
  digitalWrite(devices[deviceId].stepperPins[3], LOW);
}

void RunDeviceSteps(int deviceId)
{
  if (pendingSteps[deviceId] == 0)
  {
    return;
  }
  else if (pendingSteps[deviceId] > 0)
  {
    DebugSay(F("Advance 1 of %d pending steps.\n"), pendingSteps[deviceId]);
    steppers[deviceId]->step(1);
    --pendingSteps[deviceId];
  }
  else // in hall reset cycle
  {
    if (digitalRead(devices[deviceId].hallPin) == LOW)
    {
      DebugSay(F("Hall LOW, advance 1 step.\n"));
      steppers[deviceId]->step(1);
      lastHallReading[deviceId] = LOW;
    }
    else
    {
      if (lastHallReading[deviceId] == LOW)
      {
        DebugSay(F("Hall LOW->HIGH, advance to hallOffset of %d steps.\n"), devices[deviceId].hallOffset);
        pendingSteps[deviceId] = devices[deviceId].hallOffset;
      }
      else
      {
        DebugSay(F("Hall HIGH->HIGH, advance 1 step.\n"));
        steppers[deviceId]->step(1);
      }
      lastHallReading[deviceId] = HIGH;
    }
  }

  // did we reach a new flap?
  if (pendingSteps[deviceId] == 0)
  {
    ++currentFlap[deviceId];
    if (currentFlap[deviceId] == config.flapsPerRevolution)
    {
      currentFlap[deviceId] = 0;
    }
    if (currentFlap[deviceId] == targetFlap[deviceId])
    {
      TurnOffMotor(deviceId);
    }
  }
}

void RunDevice(int deviceId)
{
  RunDeviceSteps(deviceId);

  if (pendingSteps[deviceId] == 0 &&
      currentFlap[deviceId] != targetFlap[deviceId])
  {
    SetStepsForNextFlap(deviceId);
  }
}

void SetStepsForNextFlap(int deviceId)
{
  if (currentFlap[deviceId] == config.flapsPerRevolution - 1)
  {
    // on last flap, advance via reset mechanism
    pendingSteps[deviceId] = -1;
  }
  else
  {
    // advance normally by adding pending steps
    pendingSteps[deviceId] = config.stepsPerFlap;
  }
}

int CharacterIndex(const char ch)
{
  switch (ch)
  {
    case ' ': return 0;
    case 'A': return 1;
    //case 'Ă': return 2;
    //case 'Â': return 3;
    case 'B': return 4;
    case 'C': return 5;
    case 'D': return 6;
    case 'E': return 7;
    case 'F': return 8;
    case 'G': return 9;
    case 'H': return 10;
    case 'I': return 11;
    //case 'Î': return 12;
    case 'J': return 13;
    case 'K': return 14;
    case 'L': return 15;
    case 'M': return 16;
    case 'N': return 17;
    case 'O': return 18;
    case 'P': return 19;
    case 'Q': return 20;
    case 'R': return 21;
    case 'S': return 22;
    //case 'Ș': return 23;
    case 'T': return 24;
    //case 'Ț': return 25;
    case 'U': return 26;
    case 'V': return 27;
    case 'W': return 28;
    case 'X': return 29;
    case 'Y': return 30;
    case 'Z': return 31;
    case '0': return 32;
    case '1': return 33;
    case '2': return 34;
    case '3': return 35;
    case '4': return 36;
    case '5': return 37;
    case '6': return 38;
    case '7': return 39;
    case '8': return 40;
    case '9': return 41;
    case ',': return 42;
    case '.': return 43;
    case '!': return 44;
    case '?': return 45;
    case '#': return 46;
    case '-': return 47;
    case '+': return 48;
    case '=': return 49;
    case '@': return 50;
    case '&': return 51;
    case '/': return 52;
    case '"': return 53;
  }
}

void InitializeConfigServer()
{
  IPAddress ipAddress(config.ip[0], config.ip[1], config.ip[2], config.ip[3]);

  Ethernet.init(10);
  Ethernet.begin(config.mac, ipAddress);
  configServer.begin();
}

void RunConfigServer()
{
  EthernetClient client = configServer.available();
  if(client)
  {
    if(!telnetClientConnected)
    {
      client.flush();
      TelnetSay(F("  __ _                   _ _           _\r\n"));
      TelnetSay(F(" / _| | __ _ _ __     __| (_)___ _ __ | | __ _ _   _\r\n"));
      TelnetSay(F("| |_| |/ _` | '_ \\   / _` | / __| '_ \\| |/ _` | | | |\r\n"));
      TelnetSay(F("|  _| | (_| | |_) | | (_| | \\__ \\ |_) | | (_| | |_| |\r\n"));
      TelnetSay(F("|_| |_|\\__,_| .__/___\\__,_|_|___/ .__/|_|\\__,_|\\__, |\r\n"));
      TelnetSay(F("            |_| |_____|         |_|            |___/\r\n"));
      TelnetSay(F("\r\n> "));
      telnetClientConnected = true;
    }
    if (client.available() > 0)
    {
      char thisChar = client.read();
      if ((thisChar == '\r' || thisChar == '\n') && currentInput != "")
      {
        currentInput.toUpperCase();
        currentInput.trim();
        InterpretInput();
        currentInput = "";
        TelnetSay(F("> "));
      }
      else if (isAlphaNumeric(thisChar) || thisChar == ' ')
      {
        currentInput += thisChar;
      }
    }
  }
}

void InterpretInput()
{
  if (currentInput == "HELP")
  {
    TelnetSay(F("The following commands are available:\r\n"));
    TelnetSay(F(" CALI - calibrate the Hall effect sensor offset\r\n"));
    TelnetSay(F(" DISP - display a message on the flap display\r\n"));
    TelnetSay(F(" GOTO - make a flap go to a specific position\r\n"));
    TelnetSay(F(" HELP - display this help message\r\n"));
    TelnetSay(F(" SAVE - save the current operating configuration\r\n"));
    TelnetSay(F(" SET  - set an operating parameter\r\n"));
    TelnetSay(F(" SHOW - show operating configuration\r\n"));
    TelnetSay(F("Type 'help <command>' for more detailed information about a specific command.\r\n"));
  }
  else if (currentInput == "HELP CALI")
  {
    TelnetSay(F("The CALI command runs a calibration cycle for a specific device.\r\n"));
    TelnetSay(F("Syntax: CALI <device_id>\r\n"));
  }
  else if (currentInput == "HELP DISP")
  {
    TelnetSay(F("The DISP command displays the given message on the flap display.\r\n"));
    TelnetSay(F("Syntax: DISP <message>\r\n"));
    TelnetSay(F("All characters after the first 5 characters of the command ('DISP ') are\r\n"));
    TelnetSay(F("interpreted as the message, and do not need to be surrounded with quotes.\r\n"));
  }
  else if (currentInput == "HELP GOTO")
  {
    TelnetSay(F("The GOTO command makes a specific motor go to a specific position.\r\n"));
    TelnetSay(F("Syntax: GOTO <device_id> <position>\r\n"));
  }
  else if (currentInput == "HELP HELP")
  {
    TelnetSay(F("The HELP command displays a help message or information about a specific command.\r\n"));
    TelnetSay(F("Syntax: HELP [command]\r\n"));
  }
  else if (currentInput == "HELP SAVE")
  {
    TelnetSay(F("The SAVE command writes the current operating configuration to persistent storage.\r\n"));
    TelnetSay(F("Syntax: SAVE"));
  }
  else if (currentInput == "HELP SET")
  {
    TelnetSay(F("The SET command sets the value of an operating parameter.\r\n"));
    TelnetSay(F("Changes to parameters are not persistent. Use the SAVE command "));
    TelnetSay(F("to save the configuration to persistent storage.\r\n"));
    TelnetSay(F("Most settings take effect when restarting the unit.\r\n"));
    TelnetSay(F("Available syntaxes:\r\n"));
    TelnetSay(F("  SET MAC[0-5] <value> - sets a byte of the MAC address\r\n"));
    TelnetSay(F("  SET IP[0-3] <value> - sets a byte of the IP address\r\n"));
    TelnetSay(F("  SET SPR <steps> - sets the number of steps per revolution\r\n"));
    TelnetSay(F("  SET SPF <steps> - sets the number of steps per flap\r\n"));
    TelnetSay(F("  SET FPR <flaps> - sets the number of flaps per revolution\r\n"));
    TelnetSay(F("  SET SS <speed> - sets the stepper speed\r\n"));
    TelnetSay(F("  SET ND <value> - sets the number of devices. "));
    TelnetSay(F("This opertation will reset ALL existing device parameters\r\n"));
    TelnetSay(F("  SET DEV <device> <pin0> <pin1> <pin2> <pin3> <hall_pin> "));
    TelnetSay(F("<hall_offset> - set device parameters\r\n"));
  }
  else if (currentInput == "HELP SHOW")
  {
    TelnetSay(F("The SHOW command displays a list of all operating parameters.\r\n"));
    TelnetSay(F("Syntax: SHOW\r\n"));
  }
  else if (currentInput.startsWith("CALI ", 0))
  {
    CommandCali();
  }
  else if (currentInput.startsWith("DISP ", 0))
  {
    CommandDisp();
  }
  else if (currentInput.startsWith("GOTO ", 0))
  {
    CommandGoto();
  }
  else if (currentInput == "SAVE")
  {
    CommandSave();
  }
  else if (currentInput.startsWith("SET ", 0))
  {
    CommandSet();
  }
  else if (currentInput == "SHOW")
  {
    CommandShow();
  }
  else
  {
    TelnetSay(F("Unrecognized command or invalid syntax: '%s'.\r\nType 'help' for a list of available commands.\r\n"), currentInput.c_str());
  }
}

void CommandCali()
{
  TelnetSay(F("Not yet implemented.\r\n"));
  return;

  String idStr = currentInput.substring(5);
  byte id;
  if (idStr[0] == '0' && idStr.length() == 1)
  {
    id = 0;
  }
  else
  {
    id = idStr.toInt();
    if (id == 0)
    {
      TelnetSay(F("Unrecognized device id '%s'.\r\n"), idStr.c_str());
    }
  }
  TelnetSay(F("Zeroing device..."));
}

void CommandDisp()
{
  if (currentInput.length() <= 5)
  {
    TelnetSay(F("Invalid syntax, missing message.\r\n"));
    return;
  }

  String msgStr = currentInput.substring(5);
  if (msgStr.length() > config.numDevices)
  {
    TelnetSay(F("Message exceeds number of configured devices and will be truncated.\r\n"));
  }
  for (int d = 0; d < config.numDevices; ++d)
  {
    targetFlap[d] = CharacterIndex(msgStr[d]);
  }
}

void CommandGoto()
{
  int id, pos;
  int matches = sscanf(currentInput.c_str(), "GOTO %d %d", &id, &pos);
  if (pos != 2)
  {
    TelnetSay(F("Invalid syntax parsing GOTO command."));
    return;
  }
  targetFlap[id] = pos;
}

void CommandSave()
{
  WriteConfiguration();
  TelnetSay(F("Current configuration stored to EEPROM.\r\n"));
}

template <typename T>
void SetConfigFromInput(T *variable, int prefixLength)
{
  String str = currentInput.substring(prefixLength);
  long val = str.toInt();
  if (val == 0)
  {
    TelnetSay(F("Error parsing value.\r\n"));
  }
  else
  {
    *variable = val;
  }
}

void CommandSet()
{
  if (currentInput.startsWith("SET MAC0 "))
  {
    SetConfigFromInput(&config.mac[0], 9);
  }
  else if (currentInput.startsWith("SET MAC1 "))
  {
    SetConfigFromInput(&config.mac[1], 9);
  }
  else if (currentInput.startsWith("SET MAC2 "))
  {
    SetConfigFromInput(&config.mac[2], 9);
  }
  else if (currentInput.startsWith("SET MAC3 "))
  {
    SetConfigFromInput(&config.mac[3], 9);
  }
  else if (currentInput.startsWith("SET IP0 "))
  {
    SetConfigFromInput(&config.ip[0], 8);
  }
  else if (currentInput.startsWith("SET IP1 "))
  {
    SetConfigFromInput(&config.ip[1], 8);
  }
  else if (currentInput.startsWith("SET IP2 "))
  {
    SetConfigFromInput(&config.ip[2], 8);
  }
  else if (currentInput.startsWith("SET IP3 "))
  {
    SetConfigFromInput(&config.ip[3], 8);
  }
  else if (currentInput.startsWith("SET SPR "))
  {
    SetConfigFromInput(&config.stepsPerRevolution, 8);
  }
  else if (currentInput.startsWith("SET SPF "))
  {
    SetConfigFromInput(&config.stepsPerFlap, 8);
  }
  else if (currentInput.startsWith("SET FPR "))
  {
    SetConfigFromInput(&config.flapsPerRevolution, 8);
  }
  else if (currentInput.startsWith("SET SS "))
  {
    SetConfigFromInput(&config.stepperSpeed, 7);
  }
  else if (currentInput.startsWith("SET ND "))
  {
    SetConfigFromInput(&config.numDevices, 7);
    ReallocateDeviceArrays();
  }
  else if (currentInput.startsWith("SET DEV "))
  {
    DeviceConfig cfg;
    int devIndex;
    int matches = sscanf(currentInput.c_str(),
                         "SET DEV %d %d %d %d %d %d %d",
                         &devIndex,
                         &cfg.stepperPins[0], &cfg.stepperPins[1], &cfg.stepperPins[2], &cfg.stepperPins[3],
                         &cfg.hallPin, &cfg.hallOffset);
    if (matches == 7)
    {
      if (devIndex < config.numDevices)
      {
        devices[devIndex] = cfg;
      }
      else
      {
        TelnetSay("Invalid device number.\r\n");
      }
    }
    else
    {
      TelnetSay("Error parsing values.\r\n");
    }
  }
  else
  {
    TelnetSay(F("Invalid parameter or not yet implemented.\r\n"));
  }
}

void CommandShow()
{
  TelnetSay(F("macAddress: %X:%X:%X:%X:%X:%X\r\n"), config.mac[0], config.mac[1], config.mac[2], config.mac[3], config.mac[4], config.mac[5]);
  TelnetSay(F("ipAddress: %d.%d.%d.%d\r\n"), config.ip[0], config.ip[1], config.ip[2], config.ip[3]);

  TelnetSay(F("\r\nstepsPerRevolution: %d\r\n"), config.stepsPerRevolution);
  TelnetSay(F("stepsPerFlap: %d\r\n"), config.stepsPerFlap);
  TelnetSay(F("flapsPerRevolution: %d\r\n"), config.flapsPerRevolution);
  TelnetSay(F("stepperSpeed: %d\r\n"), config.stepperSpeed);

  if (config.numDevices == 0)
  {
    TelnetSay(F("\r\nNo devices are configured. Use SET ND to set the number of devices.\r\n"));
  }
  else
  {
    for (int i = 0; i < config.numDevices; ++i)
    {
      TelnetSay(F("\r\nDevice %d\r\n"), i);
      TelnetSay(F("  motorPins: %d %d %d %d\r\n"), devices[i].stepperPins[0], devices[i].stepperPins[1],
                                                 devices[i].stepperPins[2], devices[i].stepperPins[3]);
      TelnetSay(F("  hallPin: %d\r\n"), devices[i].hallPin);
      TelnetSay(F("  hallOffset: %d\r\n"), devices[i].hallOffset);
    }
  }
}

void CheckResetPin()
{
  static const byte resetPin = 2;
  pinMode(resetPin, INPUT_PULLUP);
  if (digitalRead(resetPin) == LOW)
  {
    FactoryReset();
    pinMode(LED_BUILTIN, OUTPUT);
    bool led = true;
    for(;;)
    {
      digitalWrite(LED_BUILTIN, led ? HIGH : LOW);
      led = !led;
      delay(1000);
    }
  }
}

void setup()
{
#ifdef ENABLE_DEBUG
  Serial.begin(2000000);
#endif

  CheckResetPin();
  ReadConfiguration();
  InitializeConfigServer();
  InitializeSteppers();
  DebugSay(F("Setup done.\n"));
}

void loop()
{
  RunConfigServer();
  for (int d = 0; d < config.numDevices; ++d)
  {
    RunDevice(d);
  }
}
