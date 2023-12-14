#include "Si115X.h"

Si115X si1151;

/**
 * Setup for configuration
 */
void setup()
{
    Wire.begin();
    Serial.begin(115200);
    if (!si1151.Begin()) {
        Serial.println("Si1151 is not ready!");
        while (1) {
            delay(1000);
            Serial.print(".");
        };
    }
    else {
        Serial.println("Si1151 is ready!");
    }
}

/**
 * Loops and reads data from registers
 */
void loop()
{
    Serial.print("IR: ");
    Serial.println(si1151.ReadIR());
    Serial.print("Visible: ");
    Serial.println(si1151.ReadVisible());

    delay(500);
}