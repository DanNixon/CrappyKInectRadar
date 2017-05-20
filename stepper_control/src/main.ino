#include <AccelStepper.h>

#define EN_PIN 2
#define STEP_PIN 3
#define DIR_PIN 4
#define MS_PIN A5

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

bool moveDone = true;

void setup()
{
  Serial.begin(9600);

  pinMode(MS_PIN, OUTPUT);
  digitalWrite(MS_PIN, HIGH);

  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(true, true, true);

  stepper.setMaxSpeed(500.0f);
  stepper.setAcceleration(1000.0f);

  stepper.enableOutputs();

  Serial.println("RDY");
}

void loop()
{
  if (Serial.available() > 2)
  {
    int delta = Serial.parseInt();
    stepper.move(delta);
    moveDone = false;
  }

  if (!moveDone && !stepper.isRunning())
  {
    Serial.println("MD");
    moveDone = true;
  }

  stepper.run();
}
