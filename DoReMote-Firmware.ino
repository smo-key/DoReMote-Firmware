#include "ble/BLE.h"
#include "ble/services/DeviceInformationService.h"

/* Bluetooth Low Energy API Setup */
BLE ble;
DeviceInformationService *deviceInfo;

static const char DEVICE_NAME[] = "Arthur's DoReMote";
static const uint16_t uuid16_list[] = {GattService::UUID_DEVICE_INFORMATION_SERVICE};

/* Pin setup */
static const int LED_INTERNAL = D13;
static const int LED_R = D3;
static const int LED_G = D0;
static const int LED_B = D1;
static const int BTN_PLAY = D7;
static const int BTN_PREV = D6;
static const int BTN_NEXT = D5;
static const int BTN_PAIR = D4;
static const int POTENTIOMETER = A5;

/* Instantiate sub-process tasks */
//Ticker ticker_pinrefresh;
Ticker ticker_debugled;

/* Instance variables */
static boolean connected = false; //true if device conneted, false otherwise
static boolean debugled = false; //state of debug led

void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{  
    Serial1.println("Disconnected. Advertising restarted!");
    connected = false;
    ble.startAdvertising();
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{  
    Serial1.println("Connected!");
    connected = true;
}

void task_debugled()
{
    //Do debug things
    debugled = !debugled;
    digitalWrite(LED_INTERNAL, debugled);
    return;
}

void handle_pairButton()
{
    Serial1.println("Hit pair button handle!");
    
    //If the device has not connected, continue
    if (!connected) { return; }

    //If the device has already connected, disconnect, clear pairs and restart advertising
    //Tell the other host that we terminated the connection
    //ble.disconnect(Gap::LOCAL_HOST_TERMINATED_CONNECTION);
    //ble.purgeAllBondingState();
}

void setup() {
    /* Instantiate debug port */
    Serial1.begin(9600);

    /* Set up pins */
    //pinMode(BTN_PLAY, INPUT);
    //pinMode(BTN_PREV, INPUT);
    //pinMode(BTN_NEXT, INPUT);
    pinMode(BTN_PAIR, INPUT);
    //pinMode(LED_R, OUTPUT);
    //pinMode(LED_G, OUTPUT);
    //pinMode(LED_B, OUTPUT);
    //pinMode(LED_INTERNAL, OUTPUT);
  
    /* Set up sub-tasks that run every time frame - times in microseconds (1000000 us = 1 s) */
    ticker_debugled.attach_us(task_debugled, 1000000);

    /* Set up interrupts that activate when input state changes */
    attachInterrupt(BTN_PAIR, handle_pairButton, CHANGE); //RISING, FALLING, OR CHANGE

    /* Initialize BLE device */
    //Initialize lower-level API.
    ble.init();
    //Allow connections. Passcode can also be specified here.
    ble.initializeSecurity();

    /* Setup services */
    //Tell devices what this is.
    deviceInfo = new DeviceInformationService(ble, "ARM", "Model1", "SN1", "hw-rev1", "fw-rev1", "soft-rev1");

    /* Setup callbacks */
    ble.gap().onDisconnection(disconnectionCallback);
    ble.gap().onConnection(connectionCallback);
    //ble.onDataWritten(writtenHandle);
    
    /* Setup BLE advertising information */
    //Enforce Bluetooth 4.0 Low Energy (BREDR_NOT_SUPPORTED) and allow discovery of device (LE_GENERAL_DISCOVERABLE).
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    //Allow connections
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    //Set a device type.
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_REMOTE_CONTROL);
    //Give device a name to broadcast.
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    //Add services list
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
   
    /* Begin advertising */
    //Set transmit power, valid values are -40, -20, -16, -12, -8, -4, 0, 4.
    ble.setTxPower(0);
    //Set advertising frequency in milliseconds.
    ble.gap().setAdvertisingInterval(1000);
    //Set advertising timeout in milliseconds. 0 is never.
    ble.setAdvertisingTimeout(0);
    //Actually begin advertising
    ble.gap().startAdvertising();

    Serial1.println("Now advertising!");
}

void loop() {
    //Wait for events (uses almost no power!)
    ble.waitForEvent();
}
