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
#define CONVERT_BUF_LEN 8
Timeout timeout;
static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state=0;
uint8_t tx_value[TXRX_BUF_LEN] = {0,};
uint8_t rx_value[TXRX_BUF_LEN] = {0,};

/* Pin setup */
static const int LED_INTERNAL = 13;
static const int LED_R = 2;
static const int LED_G = 3;
static const int LED_B = A3;
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
static boolean justconnected = false; //true if device conneted since last update, false otherwise
static boolean debugled = LOW; //state of debug led
static boolean colorlocked = false; //true if the led color is currently locked by a thread
static boolean shouldblink = false; //blink timer - flops every 2.5 s for an LED that should illuminate every 5 s
static int volume = 0; //current volume, from 0 to 100

void sendString(const char *str)
{
    uartService->writeString(str);
}
void sendLine(const char *str)
{
    uartService->writeString(str);
    uartService->writeString("\n");
}
char _int2str[7];
char* int2str( register int i ) {
  register unsigned char L = 1;
  register char c;
  register boolean m = false;
  register char b;  // lower-byte of i
  // negative
  if ( i < 0 ) {
    _int2str[ 0 ] = '-';
    i = -i;
  }
  else L = 0;
  // ten-thousands
  if( i > 9999 ) {
    c = i < 20000 ? 1
      : i < 30000 ? 2
      : 3;
    _int2str[ L++ ] = c + 48;
    i -= c * 10000;
    m = true;
  }
  // thousands
  if( i > 999 ) {
    c = i < 5000
      ? ( i < 3000
          ? ( i < 2000 ? 1 : 2 )
          :   i < 4000 ? 3 : 4
        )
      : i < 8000
        ? ( i < 6000
            ? 5
            : i < 7000 ? 6 : 7
          )
        : i < 9000 ? 8 : 9;
    _int2str[ L++ ] = c + 48;
    i -= c * 1000;
    m = true;
  }
  else if( m ) _int2str[ L++ ] = '0';
  // hundreds
  if( i > 99 ) {
    c = i < 500
      ? ( i < 300
          ? ( i < 200 ? 1 : 2 )
          :   i < 400 ? 3 : 4
        )
      : i < 800
        ? ( i < 600
            ? 5
            : i < 700 ? 6 : 7
          )
        : i < 900 ? 8 : 9;
    _int2str[ L++ ] = c + 48;
    i -= c * 100;
    m = true;
  }
  else if( m ) _int2str[ L++ ] = '0';
  // decades (check on lower byte to optimize code)
  b = char( i );
  if( b > 9 ) {
    c = b < 50
      ? ( b < 30
          ? ( b < 20 ? 1 : 2 )
          :   b < 40 ? 3 : 4
        )
      : b < 80
        ? ( i < 60
            ? 5
            : i < 70 ? 6 : 7
          )
        : i < 90 ? 8 : 9;
    _int2str[ L++ ] = c + 48;
    b -= c * 10;
    m = true;
  }
  else if( m ) _int2str[ L++ ] = '0';
  // last digit
  _int2str[ L++ ] = b + 48;
  // null terminator
  _int2str[ L ] = 0;  
  return _int2str;
}  

void safeAnalogWrite(int pin, int value)
{
  analogWrite(pin, min(max(value, 0), 255));
}
void shutoffLED()
{
  analogWrite(LED_R, 255);
  analogWrite(LED_G, 255);
  analogWrite(LED_B, 255);
}

void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{  
    Serial.println("Disconnected. Advertising restarted!");
    connected = false;
    ble.startAdvertising();

    //wait for color to unlock
    while (colorlocked && !connected)
      delay(50);

    //fade from red to black
    colorlocked = true;
   
    analogWrite(LED_R, 0);
    analogWrite(LED_G, 255);
    analogWrite(LED_B, 255);
    if (!connected)
      delay(2000);

    int i = 0;
    while (!connected && i <= 256)
    {
      safeAnalogWrite(LED_R, i);
      analogWrite(LED_G, 255);
      analogWrite(LED_B, 255);
      delay(20);
      i+= 8;
    }
    shutoffLED();

    //unlock the LED so that another thread can write to it
    colorlocked = false;
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{  
    Serial.println("Connected!");
    connected = true;
    delay(500);
    justconnected = true;
    task_pinupdate();

    //fade to green
    colorlocked = true;
    int i = 0;
    shutoffLED();
    while (connected && i <= 256)
    {
      analogWrite(LED_R, 255);
      safeAnalogWrite(LED_G, 255 - i);
      analogWrite(LED_B, 255);
      delay(20);
      i+= 8;
    }

    //wait 2 seconds
    colorlocked = false;
    if (connected)
      delay(2000);

    i = 0;
    colorlocked = true;
    //fade back to black
    while (connected && i <= 256)
    {
      analogWrite(LED_R, 255);
      safeAnalogWrite(LED_G, i);
      analogWrite(LED_B, 255);
      delay(20);
      i+= 8;
    }
    shutoffLED();

    //unlock the LED so that another thread can write to it - and start blinking connected LED
    shouldblink = true;
    colorlocked = false;
}

void task_debugled()
{   
    debugled = !debugled;
    digitalWrite(LED_INTERNAL, debugled);

    //fade in and out between blue if connected - orange if not connected and advertising
    //also, only blink the connected LED every 2 times - so alternate shouldblink
    if (connected)
      shouldblink = !shouldblink;
    if (connected && !shouldblink)
      return;
    if (colorlocked)
      return;

    shutoffLED();
    int i = 0;
    while (!colorlocked && i <= 255)
    {
      safeAnalogWrite(LED_R, connected ? 255 : 255 - i);
      safeAnalogWrite(LED_G, connected ? 255 : 255 - i/2);
      safeAnalogWrite(LED_B, connected ? 255 - i : 255);
      delay(10);
      i+= 4;
    }
    i = 0;
    while (!colorlocked && i < 255)
    {
      safeAnalogWrite(LED_R, connected ? 255 : i);
      safeAnalogWrite(LED_G, connected ? 255 : 127 + i/2);
      safeAnalogWrite(LED_B, connected ? i : 255);
      delay(10);
      i+= 4;
    }
    shutoffLED();
}

void task_pinupdate() {
    /* Update potentiometer volume value */
    //Serial.print("Potentiometer (raw): ");
    int potentiometer_raw = analogRead(POTENTIOMETER);
    //Serial.print(potentiometer_raw);
    //Serial.print(" Volume: ");
    double v = ((double)potentiometer_raw - 10.0)/1014.0;
    v = 1.0 - min(1.0, max(0.0, v));
    int vcur = (int)(v * 100); //current volume
    if (vcur != volume || justconnected)
    {
      sendString("!v ");
      sendLine(int2str(vcur));
    }
    volume = vcur;
    justconnected = false;
    //Serial.print(volume);   
}

void handle_pairButton()
{
    Serial.println("Hit pair button handle!");

    //If the device has already connected, disconnect, clear pairs and restart advertising
    //Tell the other host that we terminated the connection
    ble.disconnect(Gap::LOCAL_HOST_TERMINATED_CONNECTION);
    //ble.purgeAllBondingState();
}

void handle_playButton()
{
    Serial.println("Hit play/pause button handle!");
    sendLine("!p");
}

void handle_prevButton()
{
    Serial.println("Hit previous button handle!");
    sendLine("!<");
}

void handle_nextButton()
{
    Serial.println("Hit next button handle!");
    sendLine("!>");
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
    Serial.attach(uart_handle);

    /* Set up pins */
    pinMode(BTN_PLAY, INPUT);
    pinMode(BTN_PREV, INPUT);
    pinMode(BTN_NEXT, INPUT);
    pinMode(BTN_PAIR, INPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(LED_INTERNAL, OUTPUT);

    /* RGB LED is common cathode (+) so HIGH = OFF */
    shutoffLED();
  
    /* Set up sub-tasks that run every time frame - times in microseconds (1000000 us = 1 s) */
    ticker_debugled.attach_us(task_debugled, 2500000);
    ticker_pinupdate.attach_us(task_pinupdate, 100000);

    /* Set up interrupts that activate when input state changes */
    attachInterrupt(BTN_PAIR, handle_pairButton, RISING); //RISING, FALLING, OR CHANGE
    attachInterrupt(BTN_PLAY, handle_playButton, RISING);
    attachInterrupt(BTN_PREV, handle_prevButton, RISING);
    attachInterrupt(BTN_NEXT, handle_nextButton, RISING);

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
