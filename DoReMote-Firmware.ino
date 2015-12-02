#include "ble/BLE.h"
#include "ble/services/DeviceInformationService.h"

BLE                                 	    ble;
Timeout                                   timeout;                

const static char     DEVICE_NAME[]        = "Arthur's DoReMote";
static const uint16_t uuid16_list[]        = {GattService::UUID_DEVICE_INFORMATION_SERVICE};

DeviceInformationService *deviceInfo;

static void disconnectionCallBack(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
    Serial1.println("Disconnected!");
    Serial1.println("Now advertising!");
    ble.startAdvertising();
}

void setup() {
    // put your setup code here, to run once
    Serial1.begin(9600);

    /* Initialize BLE device */
    //Initialize lower-level API.
    ble.init();
    //Allow connections. Passcode can also be specified here.
    ble.initializeSecurity();

    /* Setup services */
    //Tell devices what this is.
    deviceInfo = new DeviceInformationService(ble, "ARM", "Model1", "SN1", "hw-rev1", "fw-rev1", "soft-rev1");

    /* Setup callbacks */
    //Restart communication on disconnection
    ble.gap().onDisconnection(disconnectionCallBack);
    
    //ble.onConnection(connectionCallback);
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
