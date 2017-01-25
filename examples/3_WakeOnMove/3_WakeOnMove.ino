
#include "Particle.h"

#include "AssetTracker.h"

// Project Location:


// System threading is required for this project
SYSTEM_THREAD(ENABLED);

// Global objects
FuelGauge batteryMonitor;
AssetTracker t;

// This is the name of the Particle event to publish for battery or movement detection events
// It is a private event.
const char *eventName = "accel";

// Various timing constants
const unsigned long MAX_TIME_TO_PUBLISH_MS = 60000; // Only stay awake for 60 seconds trying to connect to the cloud and publish
const unsigned long TIME_AFTER_PUBLISH_MS = 4000; // After publish, wait 4 seconds for data to go out
const unsigned long TIME_AFTER_BOOT_MS = 5000; // At boot, wait 5 seconds before going to sleep again (after coming online)
const unsigned long TIME_PUBLISH_BATTERY_SEC = 4 * 60 * 60; // every 4 hours, send a battery update

const uint8_t movementThreshold = 16;

// Stuff for the finite state machine
enum State { ONLINE_WAIT_STATE, RESET_STATE, RESET_WAIT_STATE, PUBLISH_STATE, SLEEP_STATE, SLEEP_WAIT_STATE, BOOT_WAIT_STATE };
State state = ONLINE_WAIT_STATE;
unsigned long stateTime = 0;
int awake = 0;

void setup() {
	Serial.begin(9600);
	t.begin();
}

void loop() {

	switch(state) {
	case ONLINE_WAIT_STATE:
		if (Particle.connected()) {
			state = RESET_STATE;
		}
		if (millis() - stateTime > 5000) {
			stateTime = millis();
			Serial.println("waiting to come online");
		}
		break;

	case RESET_STATE:
		Serial.println("resetting accelerometer");

		if (!t.setupLowPowerWakeMode(movementThreshold)) {
			Serial.println("accelerometer not found");
			state = SLEEP_STATE;
			break;
		}

		state = BOOT_WAIT_STATE;
		break;

	case PUBLISH_STATE:
		if (Particle.connected()) {
			// The publish data contains 3 comma-separated values:
			// whether movement was detected (1) or not (0) The not detected publish is used for battery status updates
			// cell voltage (decimal)
			// state of charge (decimal)
			char data[32];
			float cellVoltage = batteryMonitor.getVCell();
			float stateOfCharge = batteryMonitor.getSoC();
			snprintf(data, sizeof(data), "%d,%.02f,%.02f", awake, cellVoltage, stateOfCharge);

			Particle.publish(eventName, data, 60, PRIVATE);

			// Wait for the publish to go out
			stateTime = millis();
			state = SLEEP_WAIT_STATE;
		} else {
			// Haven't come online yet
			if (millis() - stateTime >= MAX_TIME_TO_PUBLISH_MS) {
				// Took too long to publish, just go to sleep
				state = SLEEP_STATE;
			}
		}
		break;

	case SLEEP_WAIT_STATE:
		if (millis() - stateTime >= TIME_AFTER_PUBLISH_MS) {
			state = SLEEP_STATE;
		}
		break;

	case BOOT_WAIT_STATE:
		if (millis() - stateTime >= TIME_AFTER_BOOT_MS) {
			// To publish the battery stats after boot, set state to PUBLISH_STATE
			// To go to sleep immediately, set state to SLEEP_STATE
			state = PUBLISH_STATE;
		}
		break;

	case SLEEP_STATE:
		// Sleep
		System.sleep(WKP, RISING, TIME_PUBLISH_BATTERY_SEC, SLEEP_NETWORK_STANDBY);

		// This delay should not be necessary, but sometimes things don't seem to work right
		// immediately coming out of sleep.
		delay(500);

		awake = ((t.clearAccelInterrupt() & LIS3DH_INT1_SRC_IA) != 0);

		Serial.printlnf("awake=%d", awake);

		state = PUBLISH_STATE;
		stateTime = millis();
		break;
	}
}
