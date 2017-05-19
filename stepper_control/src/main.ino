#include <AccelStepper.h>

#define EN_PIN 2
#define STEP_PIN 3
#define DIR_PIN 4
#define MS_PIN A5

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup()
{
  pinMode(MS_PIN, OUTPUT);
  digitalWrite(MS_PIN, HIGH);

  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(true, true, true);

  stepper.setMaxSpeed(500.0f);
  stepper.setAcceleration(1000.0f);

  stepper.enableOutputs();
  stepper.moveTo(1250);
}

void loop()
{
  // If at the end of travel go to the other end
  if (stepper.distanceToGo() == 0)
    stepper.moveTo(-stepper.currentPosition());

  stepper.run();
}
