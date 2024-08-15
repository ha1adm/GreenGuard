#include <GreenGuard.h>
#include <settings.h>


bool join_result;
byte ttn_payload[24];

// callback function cant take anything and return anything
Simpletimer timer1;
logging::Logger logger;
// Create Software serial instance
EspSoftwareSerial::UART swSer;
auto RN2483Serial = EspSoftwareSerial::UART(13, 15);
rn2xx3 myLora(RN2483Serial);
// Create OneWire instance with sensor
OneWire oneWire(O_WIRE);
DS18B20 SoilTS(&oneWire);
// Create BME instance with sensortype
Adafruit_BME280 bme;
// Create INA3221 instance and set I2C address to 0x41 (A0 pin -> VCC)
INA3221 ina3221(INA3221_ADDR40_GND);

//----------------------------------------------------------------------
// Init the LoRaWAN Radio modul
//----------------------------------------------------------------------
void RadioInit(int reset_pin) {
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Resetting RN2483 LoRaWAN Radio...  ");
    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, LOW);
    delay(500);
    digitalWrite(reset_pin, HIGH);
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Done");
    delay(1000); //wait for the RN2xx3's startup message
    RN2483Serial.flush();
    //check communication with radio
    String hweui = myLora.hweui();
    while(hweui.length() != 16)
    {
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "LoRaWAN", "Communication with RN2xx3 unsuccessful. Power cycle the board.");
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "LoRaWAN", hweui.c_str());
      delay(10000);
      hweui = myLora.hweui();
    }

    //print out the HWEUI
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "LoRaWAN", "When using OTAA, register this DevEUI: ");
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "LoRaWAN", myLora.hweui().c_str());
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "LoRaWAN", "RN2xx3 firmware version:");
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "LoRaWAN", myLora.sysver().c_str());
}

bool startBME(){
    unsigned status;
     status = bme.begin(0x76, &Wire);
    if (!status) {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "EnvSensor", "Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "EnvSensor", "SensorID was: 0x%d",bme.sensorID());
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "EnvSensor", "ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "EnvSensor", "ID of 0x56-0x58 represents a BMP 280");
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "EnvSensor", "ID of 0x60 represents a BME 280");
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "EnvSensor", "ID of 0x61 represents a BME 680");
        return false;
    }
    else
      return true;
}

void startINA3221(){
    // Startup INA3221
    ina3221.begin();
    ina3221.reset();
    // Set shunt resistors to 100 mOhm for all channels
    ina3221.setShuntRes(100, 100, 100);
    // Set series filter resistors to 10 Ohm for all channels.
    // Series filter resistors introduce error to the current measurement.
    // The error can be estimated and depends on the resitor values and the bus
    // voltage.
    ina3221.setFilterRes(10, 10, 10);
    // Wait for the initialisation to be end.
    delay(100);

}

void getPowerValues(){
    float current[3];
    //float current_compensated[3];
    float voltage[3];
    int16_t temp_value;

    current[0]             = ina3221.getCurrent(INA3221_CH1);
    //current_compensated[0] = ina3221.getCurrentCompensated(INA3221_CH1);
    voltage[0]             = ina3221.getVoltage(INA3221_CH1);

    current[1]             = ina3221.getCurrent(INA3221_CH2);
    //current_compensated[1] = ina3221.getCurrentCompensated(INA3221_CH2);
    voltage[1]             = ina3221.getVoltage(INA3221_CH2);

    current[2]             = ina3221.getCurrent(INA3221_CH3);
    //current_compensated[2] = ina3221.getCurrentCompensated(INA3221_CH3);
    voltage[2]             = ina3221.getVoltage(INA3221_CH3);

    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "PowerMon", "CH1 Voltage: %f",voltage[0]);
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "PowerMon", "CH1 Current: %f",current[0]);
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "PowerMon", "CH2 Voltage: %f",voltage[1]);
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "PowerMon", "CH2 Current: %f",current[1]);
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "PowerMon", "CH3 Voltage: %f",voltage[2]);
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "PowerMon", "CH3 Current: %f",current[2]);
    #ifdef LoRaWAN
      // Add Measured Values to payload
      temp_value = voltage[0]*100;
      ttn_payload[0] = temp_value >> 8;
      ttn_payload[1] = temp_value & 0xFF;
      temp_value = current[0]*100;
      ttn_payload[2] = temp_value >> 8;
      ttn_payload[3] = temp_value & 0xFF;
      temp_value = voltage[1]*100;
      ttn_payload[4] = temp_value >> 8;
      ttn_payload[5] = temp_value & 0xFF;
      temp_value = current[1]*100;
      ttn_payload[6] = temp_value >> 8;
      ttn_payload[7] = temp_value & 0xFF;
      temp_value = voltage[2]*100;
      ttn_payload[8] = temp_value >> 8;
      ttn_payload[9] = temp_value & 0xFF;
      temp_value = current[2]*100;
      ttn_payload[10] = temp_value >> 8;
      ttn_payload[11] = temp_value & 0xFF;
    #endif

}

void getEnvReadings(){
  float temperature, humidity, pressure;
  int16_t temp_value;
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = (bme.readPressure() / 100.0F);

  logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "EnvTempHumiSenor", "Temperature: %f",temperature);
  logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "EnvTempHumiSenor", "Humidity: %f",humidity);
  logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "EnvTempHumiSenor", "Pressure: %f",pressure);

  #ifdef LoRaWAN
    // Add Measured Values to payload
    temp_value = temperature*100;
    ttn_payload[12] = temp_value >> 8;
    ttn_payload[13] = temp_value & 0xFF;
    temp_value = humidity*100;
    ttn_payload[14] = temp_value >> 8;
    ttn_payload[15] = temp_value & 0xFF;
    temp_value = pressure*10;
    ttn_payload[16] = temp_value >> 8;
    ttn_payload[17] = temp_value & 0xFF;
  #endif
}

void getSoilTempReadings(){
  int16_t temp_value;
  SoilTS.requestTemperatures();
  while (!SoilTS.isConversionComplete());  // wait until sensor is ready
  float temperature = SoilTS.getTempC();
  logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "SoilTempSensor", "Temperature: %f",temperature);

  #ifdef LoRaWAN
    // Add Measured Values to payload
    temp_value = temperature*100;
    ttn_payload[18] = temp_value >> 8;
    ttn_payload[19] = temp_value & 0xFF;
  #endif

}

void getSoilMoistReadings(){
  int16_t temp_value;
  float soil_m = analogRead(A0);
  logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "SoilMoistureSensor", "Value: %f",soil_m);

  #ifdef LoRaWAN
    // Add Measured Values to payload
    temp_value = soil_m*10;
    ttn_payload[20] = temp_value >> 8;
    ttn_payload[21] = temp_value & 0xFF;
  #endif
}

void GatherAndSendData(){
  #ifdef PowerMon
    getPowerValues();
  #endif
  #ifdef EnvTempHumiSenor
    getEnvReadings();
  #endif
  #ifdef SoilTempSensor
    getSoilTempReadings();
  #endif
  #ifdef SoilMoistureSensor
    getSoilMoistReadings();
  #endif
  #ifdef LoRaWAN
  // Send it off
  switch(myLora.txBytes(ttn_payload,sizeof(ttn_payload))){
      case TX_FAIL:
      {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "LoRaWAN", "TX unsuccessful or not acknowledged");
        break;
      }
      case TX_SUCCESS:
      {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "LoRaWAN", "TX successful and acknowledged");
        break;
      }
      case TX_WITH_RX:
      {
        String received = myLora.getRx();
        //received = myLora.base16decode(received);
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "LoRaWAN", "Received downlink: ");
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "LoRaWAN", received.c_str());
        debugSerial.println(received);
        break;
      }
      default:
      {
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "LoRaWAN", "Unknown response from TX function");
      }
  }
  #endif

}

void setup() {
  debugSerial.begin(115200);
  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000);
  logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "MAIN", "Logger Started");
  RN2483Serial.begin(57600);
  RN2483Serial.setTimeout(5000);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize digital pin LOAD RELAY as an output.
  pinMode(LOAD, OUTPUT);
  digitalWrite(LOAD, HIGH);
  timer1.register_callback(GatherAndSendData);
  #ifdef PowerMon
  startINA3221();
  getPowerValues();
  #endif
  #ifdef EnvTempHumiSenor
  if(startBME()){
    getEnvReadings();
  }
  #endif
  #ifdef SoilTempSensor
  if(SoilTS.begin()){
    getSoilTempReadings();
  }
  else{
   logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "SoilTempSensor", "1-Wire Sensor init Failed!"); 
  }
  #endif
  #ifdef SoilMoistureSensor
  getSoilMoistReadings();
  #endif
  #ifdef LoRaWAN
  // Init LoRaWAN mudul
  RadioInit(LORA_RESET_P);
    #ifndef Provisioning
    join_result = false;
    join_result = myLora.initOTAA(OTAA_APPEUI, OTAA_APPKEY);
    while(!join_result)
    {
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "LoRaWAN", "Unable to join. Are your keys correct, and do you have TTN coverage?");
      delay(60000); //delay a minute before retry
      join_result = myLora.init();
    }
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "LoRaWAN", "Successfully joined TTN");
      // Set Datarate to SF7
      myLora.setDR(5);
      // Enable ADR 
      myLora.sendRawCommand(F("mac set adr on"));
      //Send first initial Packet
      GatherAndSendData();
}
    #endif
  #endif


void loop() {
  #ifndef Provisioning
    timer1.run(300000);
  #endif
}