#include "Si115X.h"

Si115X si1151;

/**
 * Setup for configuration
 */
void setup()
{
    uint8_t conf[4];

    Wire.begin();
    Serial.begin(115200);
    if (!si1151.Begin())
        Serial.println("Si1151 is not ready!");
    else
        Serial.println("Si1151 is ready!");

}

/**
 * Loops and reads data from registers
 */
void loop()
{
    Serial.print("IR: ");
    Serial.println(si1151.ReadHalfWord());
    Serial.print("VISIBLE: ");
    Serial.println(si1151.ReadHalfWord_VISIBLE());
    Serial.print("UV: ");
    Serial.println(si1151.ReadHalfWord_UV());
    delay(100);
}