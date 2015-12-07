#include "ble/BLE.h"
#include "ble/services/DeviceInformationService.h"
#define TXRX_BUF_LEN                      20

/* Bluetooth Low Energy API Setup */
BLE ble;  
DeviceInformationService *deviceInfo;

static const char DEVICE_NAME[] = "Arthur's DoReMote";
static const uint16_t uuid16_list[] = {GattService::UUID_DEVICE_INFORMATION_SERVICE};

/* UART Communication setup */
Timeout timeout; 
static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state=0;

// The Nordic UART Service
static const uint8_t service1_uuid[]                = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_tx_uuid[]             = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid[]             = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[]           = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};

uint8_t tx_value[TXRX_BUF_LEN] = {0,};
uint8_t rx_value[TXRX_BUF_LEN] = {0,};

GattCharacteristic  characteristic1(service1_tx_uuid, tx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );
GattCharacteristic  characteristic2(service1_rx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic *uartChars[] = {&characteristic1, &characteristic2};
GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));

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
Ticker ticker_debugled;
Ticker ticker_pinupdate;

/* Instance variables */
static boolean connected = false; //true if device conneted, false otherwise
static boolean debugled = false; //state of debug led
static double volume = 0.0; //current volume, from 0 to 1

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
    debugled = !debugled;
    digitalWrite(LED_INTERNAL, debugled);
    return;
}

void task_pinupdate() {
    /* Update potentiometer volume value */
    //Serial1.print("Potentiometer (raw): ");
    int potentiometer_raw = analogRead(POTENTIOMETER);
    //Serial1.print(potentiometer_raw);
    //Serial1.print(" Volume: ");
    double v = ((double)potentiometer_raw - 5.0)/1015.0;
    v = min(1.0, max(0.0, v));
    volume = v;
    //Serial1.println(volume);
}

void handle_pairButton()
{
    Serial1.println("Hit pair button handle!");
    
    //If the device has not connected, continue
    //if (!connected) { return; }

    //If the device has already connected, disconnect, clear pairs and restart advertising
    //Tell the other host that we terminated the connection
    ble.disconnect(Gap::LOCAL_HOST_TERMINATED_CONNECTION);
    ble.purgeAllBondingState();
}

/** 
 * Bluetooth communication handling
 */
void writtenHandle(const GattWriteCallbackParams *Handler)
{
    uint8_t buf[TXRX_BUF_LEN];
    uint16_t bytesRead, index;

    Serial1.println("onDataWritten : ");
    if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
        ble.readCharacteristicValue(characteristic1.getValueAttribute().getHandle(), buf, &bytesRead);
        Serial1.print("bytesRead: ");
        Serial1.println(bytesRead, HEX);
        for(byte index=0; index<bytesRead; index++) {
            Serial1.write(buf[index]);
        }
        Serial1.println("");
    }
}

void m_uart_rx_handle()
{   //update characteristic data
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), rx_buf, rx_buf_num);   
    memset(rx_buf, 0x00,20);
    rx_state = 0;
}

void uart_handle(uint32_t id, SerialIrq event)
{   /* Serial1 rx IRQ */
    if(event == RxIrq) {   
        if (rx_state == 0) {  
            rx_state = 1;
            timeout.attach_us(m_uart_rx_handle, 100000);
            rx_buf_num=0;
        }
        while(Serial1.available()) {
            if(rx_buf_num < 20) {
                rx_buf[rx_buf_num] = Serial1.read();
                rx_buf_num++;
            }
            else {
                Serial1.read();
            }
        }   
    }
}

/**
 * Setup device
 */
void setup() {
    /* Instantiate debug port */
    Serial1.begin(9600);
    Serial1.attach(uart_handle);

    /* Set up pins */
    pinMode(BTN_PLAY, INPUT);
    //pinMode(BTN_PREV, INPUT);
    //pinMode(BTN_NEXT, INPUT);
    pinMode(BTN_PAIR, INPUT);
    //pinMode(LED_R, OUTPUT);
    //pinMode(LED_G, OUTPUT);
    //pinMode(LED_B, OUTPUT);
    //pinMode(LED_INTERNAL, OUTPUT);
  
    /* Set up sub-tasks that run every time frame - times in microseconds (1000000 us = 1 s) */
    ticker_debugled.attach_us(task_debugled, 1000000);
    ticker_pinupdate.attach_us(task_pinupdate, 100000);

    /* Set up interrupts that activate when input state changes */
    attachInterrupt(BTN_PAIR, handle_pairButton, RISING); //RISING, FALLING, OR CHANGE

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
    ble.onDataWritten(writtenHandle);
    
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
    ble.setTxPower(4);
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
