/*  Converts PS4 buttons into GPIO expander controls that interface with retro game consoles NES, SNES, TG16
    ESP32 module is expected to be installed on this breakout board: https://github.com/GadgetReboot/ESP32_5V_Dev_Board

     Requires Adafruit MCP23X17 library for GPIO expander
     Requires installing bluepad32 board support https://bluepad32.readthedocs.io/en/latest/plat_arduino/

     In the tools/board menu to choose which module to program, 
     choose board type DOIT ESP32 DEV KIT V1  from the list of boards within the esp32_bluepad32 board selections 

     Tested with Arduino IDE v2.3.4
                 esp32_bluepad32 board support v4.1.0
                 Adafruit MCP23017 library v2.3.2                 

Gadget Reboot
*/

#include <Bluepad32.h>
#include <Adafruit_MCP23X17.h>

// SPI pins on ESP32 to control GPIO expander
#define CS_PIN 13
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18

#define OE_BUF_PIN 4  // Level shift buffer enable pin

Adafruit_MCP23X17 mcp;

// list of supported plug in console control boards
enum consoleList { NES = 0,
                   TG16 = 1,
                   SNES = 2,
                   UNDEFINED = 7 };
consoleList consoleType = UNDEFINED;  // default to undefined plug in board until one is detected

// plug in card ID detect ESP32 inputs
// these are pulled high by default and can be driven low by plug in cards
// to indicate what type of retro controller interface is in use
const byte ID0 = 34;
const byte ID1 = 35;
const byte ID2 = 36;  // sensor VP pin

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // configure plug in card ID inputs
  pinMode(ID0, INPUT);
  pinMode(ID1, INPUT);
  pinMode(ID2, INPUT);

  // read the plug in card ID and determine the retro controller type
  // this can surely be optimized down into one line of code but just need it to pretend to work for now...
  byte cardID = (digitalRead(ID2));
  cardID = (cardID << 1) | digitalRead(ID1);
  cardID = (cardID << 1) | digitalRead(ID0);
  consoleType = consoleList(cardID);  // set the console type based on the card ID that was read in

  switch (consoleType) {
    case NES:
      Serial.println("NES Card Detected");
      break;
    case TG16:
      Serial.println("TG16 Card Detected");
      break;
    case SNES:
      Serial.println("SNES Card Detected");
      break;
    case UNDEFINED:
      Serial.println("No Card Detected");
      break;
    default:
      break;
  }

  // enable level shift buffers
  pinMode(OE_BUF_PIN, OUTPUT);
  digitalWrite(OE_BUF_PIN, HIGH);

  // init MCP23S17 with specific SPI pins
  if (!mcp.begin_SPI(CS_PIN, SPI_SCK, SPI_MISO, SPI_MOSI)) {
    Serial.println("Fail to init.");
    while (1)
      ;
  }

  // set each GPIO expander output high (indicating no retro controller buttons are pressed)
  // and configure as outputs
  for (int i = 0; i <= 15; i++) {
    mcp.digitalWrite(i, 1);
    mcp.pinMode(i, OUTPUT);
  }

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }

  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

// show bluetooth controller data in serial monitor
void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}

// set retro controller GPIO pins based on bluetooth controller data
void processGamepad(ControllerPtr ctl) {

  // debug serial output
  // dumpGamepad(ctl);  // show gamepad status on serial monitor
  //Serial.print("Dpad: "); Serial.print(ctl->dpad());
  //Serial.print(" Buttons: "); Serial.println(ctl->buttons());

  // PS4 ctl->dpad() bits [0..3] are Up, Down, Right, Left
  // PS4 ctl->buttons() bits [0..3] are Cross, Circle, Square, Triangle

  // temporary registers to translate wireless controller button states to retro gamepad button states
  // default to all 1's (no retro input buttons pressed) and overwrite as 0 if needed
  byte gpioPortA = 0xFF;
  byte gpioPortB = 0xFF;

  // set the GPIO outputs based on what type of retro controller is in use
  switch (consoleType) {
    case NES:
      // gpio are set based on the inverted state of wireless controller data since output buttons are active low
      bitWrite(gpioPortB, 0, !bitRead(ctl->dpad(), 2));     // NES dpad Right is wireless controller dpad Right
      bitWrite(gpioPortB, 1, !bitRead(ctl->dpad(), 3));     // NES dpad Left is wireless controller dpad Left
      bitWrite(gpioPortB, 2, !bitRead(ctl->dpad(), 1));     // NES dpad Down is wireless controller dpad Down
      bitWrite(gpioPortB, 3, !bitRead(ctl->dpad(), 0));     // NES dpad Up is wireless controller dpad Up
      bitWrite(gpioPortB, 4, !bitRead(ctl->buttons(), 7));  // NES Start is wireless controller R2 Trigger
      bitWrite(gpioPortB, 5, !bitRead(ctl->buttons(), 6));  // NES Select is wireless controller L2 Trigger
      bitWrite(gpioPortB, 6, !bitRead(ctl->buttons(), 0));  // NES B button is wireless controller Cross
      bitWrite(gpioPortB, 7, !bitRead(ctl->buttons(), 1));  // NES A button is wireless controller Circle

      // write gamepad button states to gpio expander
      mcp.writeGPIOB(gpioPortB);

      // debug printout
      // Serial.print("GPIO B Port: ");
      // Serial.println(gpioPortB, BIN);

      // if R3 stick button is pressed, switch to SNES mode
      if (bitRead(ctl->buttons(), 9)) {
        consoleType = SNES;
        Serial.println("SNES Mode");
      }
      break;

    case TG16:
      // gpio are set based on the inverted state of wireless controller data since output buttons are active low
      bitWrite(gpioPortB, 0, !bitRead(ctl->buttons(), 1));  // TG16 button 2 is wireless controller Circle
      bitWrite(gpioPortB, 1, !bitRead(ctl->dpad(), 0));     // TG16 dpad Up is wireless controller dpad Up
      bitWrite(gpioPortB, 2, !bitRead(ctl->buttons(), 0));  // TG16 button 1 is wireless controller Cross
      bitWrite(gpioPortB, 3, !bitRead(ctl->dpad(), 2));     // TG16 dpad Right is wireless controller dpad Right
      bitWrite(gpioPortB, 4, !bitRead(ctl->buttons(), 2));  // TG16 Select is wireless controller Square
      bitWrite(gpioPortB, 5, !bitRead(ctl->dpad(), 1));     // TG16 dpad Down is wireless controller dpad Down
      bitWrite(gpioPortB, 6, !bitRead(ctl->buttons(), 3));  // TG16 Run is wireless controller Triangle
      bitWrite(gpioPortB, 7, !bitRead(ctl->dpad(), 3));     // TG16 dpad Left is wireless controller dpad Left

      mcp.writeGPIOB(gpioPortB);  // write gamepad button states to gpio expander port

      /*. // debug output
      Serial.print("GPIO B Port: ");
      Serial.print(gpioPortB, BIN);
      */
      break;

    case SNES:
      // gpio are set based on the inverted state of wireless controller data since output buttons are active low
      bitWrite(gpioPortB, 0, !bitRead(ctl->dpad(), 2));     // SNES dpad Right is wireless controller dpad Right
      bitWrite(gpioPortB, 1, !bitRead(ctl->dpad(), 3));     // SNES dpad Left is wireless controller dpad Left
      bitWrite(gpioPortB, 2, !bitRead(ctl->dpad(), 1));     // SNES dpad Down is wireless controller dpad Down
      bitWrite(gpioPortB, 3, !bitRead(ctl->dpad(), 0));     // SNES dpad Up is wireless controller dpad Up
      bitWrite(gpioPortB, 4, !bitRead(ctl->buttons(), 7));  // SNES Start is wireless controller R2 Trigger
      bitWrite(gpioPortB, 5, !bitRead(ctl->buttons(), 6));  // SNES Select is wireless controller L2 Trigger
      bitWrite(gpioPortB, 6, !bitRead(ctl->buttons(), 2));  // SNES Y button is wireless controller Square
      bitWrite(gpioPortB, 7, !bitRead(ctl->buttons(), 0));  // SNES B button is wireless controller Cross

      bitWrite(gpioPortA, 0, !bitRead(ctl->buttons(), 5));  // SNES Right Trigger is wireless controller R1 Trigger
      bitWrite(gpioPortA, 1, !bitRead(ctl->buttons(), 4));  // SNES Left Trigger is wireless controller L1 Trigger
      bitWrite(gpioPortA, 2, !bitRead(ctl->buttons(), 3));  // SNES X button is wireless controller Triangle
      bitWrite(gpioPortA, 3, !bitRead(ctl->buttons(), 1));  // SNES A button is wireless controller Circle

      // write gamepad button states to gpio expander
      mcp.writeGPIOA(gpioPortA);
      mcp.writeGPIOB(gpioPortB);

      /*
      // debug printout
      Serial.print("GPIO B Port: ");
      Serial.print(gpioPortB, BIN);
      Serial.print(" GPIO A Port: ");
      Serial.println(gpioPortA, BIN);
      */

      // if L3 stick button is pressed, switch to SNES mode
      if (bitRead(ctl->buttons(), 8)) {
        consoleType = NES;
        Serial.println("NES Mode");
      }
      break;

    case UNDEFINED:
      // do nothing
      break;

    default:
      break;
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void loop() {
  // This call fetches all the controller data.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
  vTaskDelay(1);
}