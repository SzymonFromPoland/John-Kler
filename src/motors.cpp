#include <motors.h>

void setup_motors()
{
    pinMode(M1A, OUTPUT);
    pinMode(M2A, OUTPUT);
    pinMode(M1B, OUTPUT);
    pinMode(M2B, OUTPUT);
    pinMode(SLP, OUTPUT);
    pinMode(OFF, OUTPUT);
    pinMode(NFA, INPUT_PULLUP);
    pinMode(NFB, INPUT_PULLUP);

    digitalWrite(SLP, HIGH);
    delay(5);

    digitalWrite(SLP, LOW);
    delayMicroseconds(50);

    digitalWrite(SLP, HIGH);
    delay(5);

    digitalWrite(OFF, LOW);
}

void drive(int motor1, int motor2)
{
    motor2 = constrain(motor2, -100, 100);
    motor1 = constrain(motor1, -100, 100);

    int pwmL = map(abs(motor2), 0, 100, 0, 255);
    int pwmR = map(abs(motor1), 0, 100, 0, 255);

    if (motor2 >= 0)
    {
        digitalWrite(M2A, LOW);
        analogWrite(M1A, pwmL);
    }
    else
    {
        digitalWrite(M2A, HIGH);
        analogWrite(M1A, pwmL);
    }

    if (motor1 >= 0)
    {
        digitalWrite(M2B, LOW);
        analogWrite(M1B, pwmR);
    }
    else
    {
        digitalWrite(M2B, HIGH);
        analogWrite(M1B, pwmR);
    }
}