#include <Servo.h>
#include <Balboa32U4.h>
#include <PololuRPiSlave.h>
#define SENSOR_COUNT 5





#include <Arduino.h>
#include <HardwareSerial.h>

#define SERIAL_PORT Serial1
#define BAUD_RATE 115200

#define DWM_ACTIVE true




/* This file is a modified version of the file written by Pololu:
 * https://github.com/pololu/pololu-rpi-slave-arduino-library
 */

struct Data
{
  // commands given by RPi
  int16_t leftMotor, rightMotor;
  // given by Balboa
  int16_t leftEncoder, rightEncoder;
  uint16_t lineSensors[SENSOR_COUNT];




  uint16_t distance;
  int16_t x, y, z;



};

PololuRPiSlave<struct Data,20> slave;
PololuBuzzer buzzer;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4LineSensors line_sensors;
uint16_t line_sensor_values[SENSOR_COUNT];

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(20);

  Serial.begin(115200);
  SERIAL_PORT.begin(BAUD_RATE);

  // Play startup sound.
  buzzer.play("v10>>g16>>>c16");
  
  SERIAL_PORT.setTimeout(5);//(30); // Each time DWM is read, the main loop will take at least 30 ms. This is bad for the balancing loop then it need to be minimized.

  line_sensors.setEdgeAligned();
}


int32_t current_time = millis();
int32_t begin_loop_time = millis();
int DWM_REFRESH_RATE_MS = 100;

void loop()
{
  unsigned long millisstart = millis();

  line_sensors.read(line_sensor_values);

  slave.updateBuffer();

  motors.setSpeeds(slave.buffer.leftMotor, slave.buffer.rightMotor);

  slave.buffer.leftEncoder = encoders.getCountsLeft();
  slave.buffer.rightEncoder = encoders.getCountsRight();

  for (int i = 0; i < SENSOR_COUNT; i++) {
    slave.buffer.lineSensors[i] = line_sensor_values[i];
  }

  if (DWM_ACTIVE && millis() - current_time > DWM_REFRESH_RATE_MS) {
    size_t len = dwmLocGet();
    current_time = millis();

    //Serial.print("time of loop: ");
    //Serial.println(millis() - millisstart);
    //Serial.print("length of message: ");
    //Serial.println(len);
  }

  slave.finalizeWrites();


  //Serial.print("Loop time: ");
  //Serial.println(millis() - begin_loop_time);
  //begin_loop_time = millis();

  //delayMicroseconds(1000); // 1 ms of delay // < 1000 Hz
}







bool verbose = false;

/* The code is originally from Polulu, and has been modified by Romain Englebert*/

void hexStr(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (data[i] < 16) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void error(uint8_t err_code) {
    switch (err_code) {
        case 0: Serial.println("OK"); break;
        case 1: Serial.println("Unknown command or broken TLV frame"); break;
        case 2: Serial.println("Internal error"); break;
        case 3: Serial.println("Invalid parameter"); break;
        case 4: Serial.println("Busy"); break;
        case 5: Serial.println("Operation not permitted"); break;
        default: Serial.println("Unknown error");
    }
}

size_t dwmLocGet() {
    const uint8_t DWM1001_TLV_TYPE_CMD_LOC_GET = 0x0C;
    const uint8_t TLV_TYPE_RET_VAL = 0x40;
    const uint8_t TLV_TYPE_POS_XYZ = 0x41;
    const uint8_t TLV_TYPE_RNG_AN_DIST = 0x48;
    const uint8_t TLV_TYPE_RNG_AN_POS_DIST = 0x49;

    const uint8_t POS_LEN = 13;
    const uint8_t DIST_LEN = 7;

    uint8_t tx_data[] = {DWM1001_TLV_TYPE_CMD_LOC_GET, 0x00};

    SERIAL_PORT.write(tx_data, sizeof(tx_data));
    
    uint8_t rx_data[200];
    size_t len = SERIAL_PORT.readBytes(rx_data, sizeof(rx_data));
    
    if (verbose) {
        Serial.print("Received: ");
        hexStr(rx_data, len);
    }
    
    size_t data_cnt = 0;
    if (rx_data[data_cnt] == TLV_TYPE_RET_VAL) {
        if (verbose) {
            Serial.println("--- received error code");
        }

        uint8_t err_code = rx_data[data_cnt + 2];
        if (verbose) {
            error(err_code);
        }
        data_cnt += 3;
    }
    
    if (rx_data[data_cnt] == TLV_TYPE_POS_XYZ) {
        if (verbose) {
            Serial.println("--- updating position");
        }

        data_cnt += 2;
        int32_t x, y, z;
        uint8_t qf;
        memcpy(&x, &rx_data[data_cnt], 4);
        memcpy(&y, &rx_data[data_cnt + 4], 4);
        memcpy(&z, &rx_data[data_cnt + 8], 4);
        qf = rx_data[data_cnt + 12];
        if (verbose) {
            Serial.print("Position: X="); Serial.print(x);
            Serial.print(", Y="); Serial.print(y);
            Serial.print(", Z="); Serial.print(z);
            Serial.print(", QF="); Serial.println(qf);
        }
        data_cnt += POS_LEN;

        slave.buffer.x = x;
        slave.buffer.y = y;
        slave.buffer.z = z;
    }

    if (rx_data[data_cnt] == TLV_TYPE_RNG_AN_POS_DIST) {
      if (verbose) {
          Serial.println("--- state of anchors + updating position to target");
      }

      uint8_t an_pos_dist_len = rx_data[data_cnt+1];
      uint8_t an_number = rx_data[data_cnt+2];
      data_cnt += 3;

      for (int i=0; i < an_number; i++) {

          if (an_pos_dist_len) {
              uint16_t d;;
              uint8_t qf;
              int16_t uwb_addr;

              memcpy(&uwb_addr, &rx_data[data_cnt], 2);
              memcpy(&d, &rx_data[data_cnt+2], 4);
              qf = rx_data[data_cnt + 6];
              if (verbose) {        
                  Serial.print("UWB addr="); Serial.print(uwb_addr, HEX);
                  Serial.print(", d="); Serial.print(d);
                  Serial.print(", QF="); Serial.println(qf);
              }

              if (uwb_addr == 0xFFFFC933) {
                slave.buffer.distance = d;
              }

              data_cnt += DIST_LEN;

              int32_t x, y, z;
              memcpy(&x, &rx_data[data_cnt], 4);
              memcpy(&y, &rx_data[data_cnt + 4], 4);
              memcpy(&z, &rx_data[data_cnt + 8], 4);
              qf = rx_data[data_cnt + 12];
              if (verbose) {
                  Serial.print("Position: X="); Serial.print(x);
                  Serial.print(", Y="); Serial.print(y);
                  Serial.print(", Z="); Serial.print(z);
                  Serial.print(", QF="); Serial.println(qf);
              }

          }
          data_cnt += POS_LEN;
      }
   }

   return len;
}
