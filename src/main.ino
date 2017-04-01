/*
    Modbus slave simple example

    Control and Read Arduino I/Os using Modbus serial connection.

    This sketch show how to use the callback vector for reading and
    controleing Arduino I/Os.

    * Controls digital output pins as modbus coils.
    * Reads digital inputs state as discreet inputs.
    * Reads analog inputs as input registers.

    The circuit: ( see: ./extras/ModbusSetch.pdf )
    * An ESP8266 unit with connectors to digital out pins.
    * 2 x LEDs, with 220 ohm resistors in series.
    * A switch connected to a digital input pin.
    * A potentiometer connected to an analog input pin.

    Created 8 12 2017
    By Yaacov Zamir

    https://github.com/yaacov/ArduinoModbusSlave

*/

#include <ESP8266WiFi.h>
//#include <ModbusSlaveTCP.h>
#include <ModbusSlave.h>
#include <ESP8266mDNS.h>
#include <dht11.h>
#include <OneWire.h>

const char* ssid = "***";
const char* pass = "***";
MDNSResponder mdns;

dht11 DHT11;

/* slave id = 1
 */
#define SLAVE_ID 1
#define DHT11PIN 2
#define LEDPIN  13
#define DS18PIN  4

/**
 *  Modbus object declaration
 */
//ModbusTCP slave(SLAVE_ID);
Modbus slave(SLAVE_ID, 0); // [stream = Serial,] slave id = 1, rs485 control-pin = 8

// DS18S20 Temperature chip i/o
OneWire ds(DS18PIN);  // on pin 10
byte addr[8];
float t2 = 0;

// read DS18S20 device
int readDS() {
    int HighByte, LowByte, TReading, SignBit, Tc_100;
    byte i, sensor;
    byte present = 0;
    byte data[12];

    // check CRC
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      //Serial.println("bad CRC");
      return 0;
    }

    if ( addr[0] != 40) {
      ////Serial.print("not DS18S20 ");
      ////Serial.println(addr[0]);
      return 0;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44,1);         // start conversion, with parasite power on at the end

    delay(250);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
    }

    LowByte = data[0];
    HighByte = data[1];
    TReading = (HighByte << 8) + LowByte;
    SignBit = TReading & 0x8000;  // test most sig bit
    if (SignBit) {
      TReading = (TReading ^ 0xffff) + 1; // 2's comp
    }

    return TReading * 100 / 2;
}

// delta max = 0.6544 wrt dewPoint()
// 6.9 x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double dewPointFast(double celsius, double humidity)
{
    double a = 17.271;
    double b = 237.7;
    double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
    double Td = (b * temp) / (a - temp);
    return Td;
}

void setup() {
    pinMode(LEDPIN, OUTPUT);

    /* Start serial port
     */
    Serial.begin(115200);

    /* Connect WiFi to the network
     */
    //Serial.print("Connecting to ");
    //Serial.println(ssid);
    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        //Serial.print(".");
    }

    /* register handler functions
     * into the modbus slave callback vector.
     */
    slave.cbVector[CB_WRITE_COIL] = writeDigitlOut;
    slave.cbVector[CB_READ_COILS] = readDigitalIn;
    slave.cbVector[CB_READ_REGISTERS] = readAnalogIn;

    /* start slave and listen to TCP port 502
     */
    //slave.begin();
    slave.begin(115200);

    // log to serial port
    //Serial.println("");
    //Serial.print("Modbus ready, listen on ");
    //Serial.println(ssid);
    //Serial.print("IP address: ");
    //Serial.println(WiFi.localIP());

    if (mdns.begin("modbus", WiFi.localIP())) {
      //Serial.println("MDNS responder started");
    }

    // look up DS18S20 device
    if (!ds.search(addr)) {
      ds.reset_search();
      delay(250);
      //Serial.println("DS18S20 not found");
    }
}

void loop() {
    /* listen for modbus commands con serial port
     *
     * on a request, handle the request.
     * if the request has a user handler function registered in cbVector
     * call the user handler function.
     */
    slave.poll();

    // read the temperature
    //t2 = (float)readDS();
}

/**
 * Handel Force Single Coil (FC=05)
 * set digital output pins (coils) on and off
 */
void writeDigitlOut(uint8_t fc, uint16_t address, uint16_t status) {
    digitalWrite(address, status);
}

/**
 * Handel Read Input Status (FC=02/01)
 * write back the values from digital in pins (input status).
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code
 *      uint16_t address - first register/coil address
 *      uint16_t length/status - length of data / coil status
 */
void readDigitalIn(uint8_t fc, uint16_t address, uint16_t length) {
    // read digital input
    for (int i = 0; i < length; i++) {
        slave.writeCoilToBuffer(i, digitalRead(address + i));
    }
}

/**
 * Handel Read Input Registers (FC=04/03)
 * write back the values from analog in pins (input registers).
 */
void readAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    int chk = DHT11.read(DHT11PIN);
    float t = 0;
    float h = 0;
    float d = 0;

    //Serial.print("Read sensor: ");
    switch (chk)
    {
        case DHTLIB_OK:
            //Serial.println("OK");
            t = (float)DHT11.temperature;
            h = (float)DHT11.humidity;
            d = dewPointFast(t, h);

            break;
        case DHTLIB_ERROR_CHECKSUM:
            //Serial.println("Checksum error");
            break;
        case DHTLIB_ERROR_TIMEOUT:
            //Serial.println("Time out error");
            break;
        default:
            //Serial.println("Unknown error");
            break;
    }

    //Serial.print("Temperature DHT (°C): ");
    //Serial.println(t);

    //Serial.print("Temperature DS18 (°C): ");
    //Serial.println(t2 / 1000.0);

    //Serial.print("Humidity (%%): ");
    //Serial.println(h);

    //Serial.print("Dew PointFast (°C): ");
    //Serial.println(d);

    for (uint16_t i = 0; i < length; i++) {
        switch (address + i)
        {
            case 0:
                slave.writeRegisterToBuffer(i, t2 / 10);
                break;
            case 1:
                slave.writeRegisterToBuffer(i, h * 100);
                break;
            case 2:
                slave.writeRegisterToBuffer(i, d * 100);
                break;
            case 3:
                slave.writeRegisterToBuffer(i, t * 100);
                break;
            default:
                slave.writeRegisterToBuffer(i, 9999);
                break;
        }
    }
}
