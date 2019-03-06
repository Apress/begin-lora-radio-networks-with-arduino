#include <SPI.h> // Serial Pheripheral Interface library
#include <RH_RF95.h> //RadioHead RFM9x library
#include "DHT.h" //DHT temperature and humidity sensor library

//Radio pinout setup
#define RFM95_CS 4 //CS pin is connected to Arduino digital pin 4
#define RFM95_RST 2 //RST pin is connected to Arduino digital pin 2
#define RFM95_INT 3 //G0 pin is connected to Arduino digital pin 3
//DHT11 sensor pinout setup
#define DHTPIN 8     // Data out pin is connected to Arduino digital pin 8

#define RF95_FREQ 433.0

#define DHTTYPE DHT11   // DHT 11 sensor

RH_RF95 rf95(RFM95_CS, RFM95_INT);

DHT dht(DHTPIN, DHTTYPE);

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("initializing…");
    while (1);
  }
  Serial.println("initialisation succeeded");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);

  dht.begin();
}

void loop()
{



  float t = dht.readTemperature();

  if (isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }


  Serial.println("Sending to receiver");
  
  char radiopacket[20]= "";
  
  dtostrf(t,5,2,radiopacket);
  Serial.print("Sending "); Serial.println(radiopacket);
  

  Serial.println("Sending..."); delay(10);
 
  rf95.send((uint8_t *) radiopacket, 20);

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  
// Waiting for the reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(1000))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
    {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is the receiver running?");
  }
  delay(1000);
}

