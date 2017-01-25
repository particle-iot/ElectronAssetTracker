/* -----------------------------------------------------------
This example shows some clever ways to use the accelerometer
and report that data efficiently. It also includes the transmit
mode and battery reading functions from the previous example.

By default, it publishes only when there's a significant bump
against it AND there's been at least 10 minutes since the last
publish. NOTE: the first publish will only happen after the
device has been on for at least this long!
---------------------------------------------------------------*/

// Getting the library
#include "AssetTracker.h"

// Set whether you want the device to publish data to the internet by default here.
// 1 will Particle.publish AND Serial.print, 0 will just Serial.print
// Extremely useful for saving data while developing close enough to have a cable plugged in.
// You can also change this remotely using the Particle.function "tmode" defined in setup()
int transmittingData = 1;

// Used to keep track of the last time we published data
long lastPublish = 0;

// How many minutes minimum between publishes? 10+ recommended!
int delayMinutes = 10;

// Threshold to trigger a publish
// 9000 is VERY sensitive, 12000 will still detect small bumps
int accelThreshold = 12000;

// Creating an AssetTracker named 't' for us to reference
AssetTracker t = AssetTracker();

// A FuelGauge named 'fuel' for checking on the battery state
FuelGauge fuel;

// setup() and loop() are both required. setup() runs once when the device starts
// and is used for registering functions and variables and initializing things
void setup() {
    // Sets up all the necessary AssetTracker bits
    t.begin();

    // Opens up a Serial port so you can listen over USB
    Serial.begin(9600);

    // These three functions are useful for remote diagnostics. Read more below.
    Particle.function("aThresh",accelThresholder);
    Particle.function("tmode", transmitMode);
    Particle.function("batt", batteryStatus);
}

// loop() runs continuously
void loop() {
    // Check if there's been a big acceleration
    if (t.readXYZmagnitude() > accelThreshold) {
        // Create a nice string with commas between x,y,z
        String pubAccel = String::format("%d,%d,%d", t.readX(), t.readY(), t.readZ());

        // Send that acceleration to the serial port where it can be read by USB
        //Serial.println(pubAccel);
        Serial.println(t.readXYZmagnitude());

        // If it's set to transmit AND it's been at least delayMinutes since the last one...
        if (transmittingData && ((millis()-lastPublish) > (delayMinutes*60*1000))) {
            lastPublish = millis();
            Particle.publish("A", pubAccel, 60, PRIVATE);
        }

    }
}

// Remotely change the trigger threshold!
int accelThresholder(String command) {
    accelThreshold = atoi(command);
    return 1;
}

// Allows you to remotely change whether a device is publishing to the cloud
// or is only reporting data over Serial. Saves data when using only Serial!
// Change the default at the top of the code.
int transmitMode(String command) {
    transmittingData = atoi(command);
    return 1;
}

// Lets you remotely check the battery status by calling the function "batt"
// Triggers a publish with the info (so subscribe or watch the dashboard)
// and also returns a '1' if there's >10% battery left and a '0' if below
int batteryStatus(String command) {
    // Publish the battery voltage and percentage of battery remaining
    // if you want to be really efficient, just report one of these
    // the String::format("%f.2") part gives us a string to publish,
    // but with only 2 decimal points to save space
    Particle.publish("B",
          "v:" + String::format("%.2f",fuel.getVCell()) +
          ",c:" + String::format("%.2f",fuel.getSoC()),
          60, PRIVATE
    );
    // if there's more than 10% of the battery left, then return 1
    if (fuel.getSoC() > 10) {
      return 1;

    // if you're running out of battery, return 0
    } else {
      return 0;
    }
}
