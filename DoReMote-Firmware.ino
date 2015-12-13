#include "ble/BLE.h"
#include "ble/services/DeviceInformationService.h"
#include "ble/services/UARTService.h"

/* Bluetooth Low Energy API Setup */
BLE ble;  
UARTService *uartService;
DeviceInformationService *deviceInfo;

static const char DEVICE_NAME[] = "Arthur's DoReMote";

/* UART Communication setup */
#define TXRX_BUF_LEN 20
Timeout timeout;
static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state=0;
uint8_t tx_value[TXRX_BUF_LEN] = {0,};
uint8_t rx_value[TXRX_BUF_LEN] = {0,};

/* Pin setup */
static const int LED_INTERNAL = 13;
static const int LED_R = 3;
static const int LED_G = 0;
static const int LED_B = 1;
static const int BTN_PLAY = 7;
static const int BTN_PREV = 6;
static const int BTN_NEXT = 5;
static const int BTN_PAIR = 4;
static const int POTENTIOMETER = A5;

/* Instantiate sub-process tasks */
Ticker ticker_debugled;
Ticker ticker_pinupdate;

/* Instance variables */
static boolean connected = false; //true if device conneted, false otherwise
static boolean debugled = LOW; //state of debug led
static double volume = 0.0; //current volume, from 0 to 1

void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{  
    Serial.println("Disconnected. Advertising restarted!");
    connected = false;
    ble.startAdvertising();
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{  
    Serial.println("Connected!");
    connected = true;
}

void task_debugled()
{   
    debugled = !debugled;
    digitalWrite(LED_INTERNAL, debugled);
}

void task_pinupdate() {
    /* Update potentiometer volume value */
    Serial.print("Potentiometer (raw): ");
    int potentiometer_raw = analogRead(POTENTIOMETER);
    Serial.print(potentiometer_raw);
    Serial.print(" Volume: ");
    double v = ((double)potentiometer_raw - 5.0)/1015.0;
    v = 1.0 - min(1.0, max(0.0, v));
    volume = v;
    Serial.print(volume);

    Serial.print(" Pair: ");
    Serial.print(digitalRead(BTN_PAIR));
    Serial.print(" Play: ");
    Serial.println(digitalRead(BTN_PLAY));
}

void handle_pairButton()
{
    Serial.println("Hit pair button handle!");
    
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

    if (Handler->handle == uartService->getTXCharacteristicHandle()) {
        ble.readCharacteristicValue(uartService->getTXCharacteristicHandle(), buf, &bytesRead);
        for(byte index=0; index<bytesRead; index++) {
            Serial.write(buf[index]);
        }
    }
}

void m_uart_rx_handle()
{   //update characteristic data
    ble.updateCharacteristicValue(uartService->getRXCharacteristicHandle(), rx_buf, rx_buf_num);
    memset(rx_buf, 0x00,20);
    rx_state = 0;
}

void uart_handle(uint32_t id, SerialIrq event)
{   /* Serial rx IRQ */
    if(event == RxIrq) {
        if (rx_state == 0) {
            rx_state = 1;
            timeout.attach_us(m_uart_rx_handle, 100000);
            rx_buf_num=0;
        }
        while(Serial.available()) {
            if(rx_buf_num < 20) {
                rx_buf[rx_buf_num] = Serial.read();
                rx_buf_num++;
            }
            else {
                Serial.read();
            }
        }
    }
}

/**
 * Setup device
 */
void setup() {
    /* Instantiate debug port */
    Serial.begin(9600);

    /* Set up pins */
    //pinMode(BTN_PLAY, INPUT);
    //pinMode(BTN_PREV, INPUT);
    //pinMode(BTN_NEXT, INPUT);
    //pinMode(BTN_PAIR, INPUT);
    //pinMode(LED_R, OUTPUT);
    //pinMode(LED_G, OUTPUT);
    //pinMode(LED_B, OUTPUT);
    pinMode(LED_INTERNAL, OUTPUT);
  
    /* Set up sub-tasks that run every time frame - times in microseconds (1000000 us = 1 s) */
    ticker_debugled.attach_us(task_debugled, 1000000);
    ticker_pinupdate.attach_us(task_pinupdate, 100000);

    /* Set up interrupts that activate when input state changes */
    //attachInterrupt(BTN_PAIR, handle_pairButton, RISING); //RISING, FALLING, OR CHANGE
    //attachInterrupt(BTN_PLAY, handle_pairButton, RISING);

    /* Initialize BLE device */
    //Initialize lower-level API.
    ble.init();
    //Allow connections. Passcode can also be specified here.
    ble.initializeSecurity();
    
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
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                    (const uint8_t *)UARTServiceUUID, sizeof(UARTServiceUUID));

    /* Setup services */
    //Tell devices what this is.
    deviceInfo = new DeviceInformationService(ble, "ARM", "Model1", "SN1", "hw-rev1", "fw-rev1", "soft-rev1");
    uartService = new UARTService(ble);
    Serial.attach(uart_handle);
    
    /* Begin advertising */
    //Set transmit power, valid values are -40, -20, -16, -12, -8, -4, 0, 4.
    ble.setTxPower(4);
    //Set advertising frequency in milliseconds.
    ble.gap().setAdvertisingInterval(1000);
    //Set advertising timeout in milliseconds. 0 is never.
    ble.setAdvertisingTimeout(0);
    //Actually begin advertising
    ble.gap().startAdvertising();

    Serial.println("Now advertising!");
}

void loop() {
    //Wait for events (uses almost no power!)
    ble.waitForEvent();
}
