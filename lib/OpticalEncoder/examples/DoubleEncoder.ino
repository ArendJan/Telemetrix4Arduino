#include <OpticalEncoder.h>

OpticalEncoder *encoders[2];
uint8_t fanpins[2] = {2, 3};
long sum_speed[2] = {0, 0};
int nr_loops = 0;

void setup()
{
  Serial.begin(115200);
  encoders[0] = new OpticalEncoder();
  encoders[1] = new OpticalEncoder();
  encoders[0]->setup(fanpins[0], []{encoders[0]->handleInterrupt();}, FALLING, 20);
  encoders[1]->setup(fanpins[1], []{encoders[1]->handleInterrupt();}, FALLING, 20);
}

void loop()
{
  static uint32_t lastMillis = 0;
  static const uint32_t minuteMillis = millis();
  if (millis() - minuteMillis < 10000UL) {
    if (millis() - lastMillis > 100UL)
    {
      sum_speed[0] += encoders[0]->getSpeed();
      sum_speed[1] += encoders[1]->getSpeed();
      nr_loops++;
      lastMillis = millis();
    }
  } else {
    Serial.println(F("AVG ENCODER 1: "));Serial.println(floor(sum_speed[0]/nr_loops + 0.5), 0);
    Serial.println(F("Last Speed ENCODER 1: "));Serial.println(floor(encoders[0]->getSpeed() + 0.5), 0);
    Serial.println(F("NrTicks ENCODER 1: "));Serial.println(encoders[0]->getPosition());

    Serial.println(F("AVG ENCODER 2: "));Serial.println(floor(sum_speed[1]/nr_loops + 0.5), 0);
    Serial.println(F("Last Speed ENCODER 2: "));Serial.println(floor(encoders[1]->getSpeed() + 0.5), 0);
    Serial.println(F("NrTicks ENCODER 2: "));Serial.println(encoders[1]->getPosition());
    while(1){};        //last line of main loop
  }
}
