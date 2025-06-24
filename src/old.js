// All Puck.JS External Modules
const Accel = require("puckjsv2-accel-movement");
const DS3231 = require("DS3231");
const Storage = require("Storage");

// Logger, in the style of WinstonJS (https://github.com/winstonjs/winston)
const logLevel = {
  off: 0,
  error: 1,
  info: 2,
  debug: 3,
  silly: 4,
};

const Logger = function (initialLevel) {
  const self = this;
  const write = function (msg, level) {
    if (level <= self.level) {
      print(msg);
    }
  };

  self.level = initialLevel || logLevel.info;

  self.silly = function (msg) {
    write(msg, logLevel.silly);
  };

  self.debug = function (msg) {
    write(msg, logLevel.debug);
  };

  self.info = function (msg) {
    write(msg, logLevel.info);
  };

  self.error = function (msg) {
    write(msg, logLevel.error);
  };
};

const logger = new Logger(logLevel.info);

// Enums
const eventType = {
  rollChange: 0,
  dispense: 1,
};

// Settings - These values are theoretically configurable.
const ADVERTISE_EVENT_DATA_INTERVAL_TIME_IN_MS = 2000;
const CHECK_ROLL_STATUS_INTERVAL_TIME_IN_MS = 1000;
const CLEAR_OLD_EVENTS_INTERVAL_TIME_IN_MS = 3600000;
const EVENT_DATA_BLE_ADVERTISING_INTERVAL_IN_MS = 1000;
const EXPIRED_EVENT_AGE_IN_SEC = 172800; // Two days
const PULL_REV_THRESHOLD = 0.1;
const PULL_TIMEOUT_IN_MS = 1000;
const REQUIRED_SPINS_AFTER_CHANGE = 0.5;
const SESSION_TIMEOUT_IN_MS = 120000;

// Constants - These values should never change and are not meant to be configurable.
// TODO: Make a constant for each Pin used
const FIRMWARE_VERSION = "1.0.0";
const KC_BLUETOOTH_COMPANY_ID = 0x03fc;

// This magic constant is used to convert the raw gyrometer value into a meaningful revolutions value.
// See this Espruino forum thread for a discussion on using the gyro to count revolutions and the value below:
//   http://forum.espruino.com/conversations/354579/#comment15674181
const PUCK_GYRO_CONSTANT = 600000;

/*
  We are not able to broadcast the full 128-bit Device ID since we do not have enough space in the advertising packet.
  Also, we do not allow BLE clients to connect to the device which would allow them to read more detailed information via a GATT service.
  To uniquely identify a device, the mobile app looks for a BLE device where the local name begins with "SS" broadcasting a datapacket.
  The app assumes the device is a Smart Spindle, and it also assumes the last two characters of the local name are the Device's full serial number.
  Since we are only creating about 20 devices, we will not run out of available serial numbers (16 * 16 = 256 possible numbers).
  It's not an ideal solution since other non-KC devices could also have a similar naming convention.
  A better solution would be to find another way to broadcast more Manufacturer data to more accurately identify the Device rather than relying on conventions and interpretations hard-coded in the app.
  We chose this solution due to time and proect constraints and since we didn't know how best to utilize BLE advertising.
*/
// Read the Device's Serial Number from a file in the flash memory; coerce to string
const serialNumber = `${Storage.read("serialNumber")}`;
logger.info(`Starting Smart Spindle S/N ${serialNumber}...  Firmware version: ${FIRMWARE_VERSION}`);

// Used when advertising event data to identify a Smart Spindle
const bleLocalName = `SS${serialNumber.substr(-2)}`;

// Module Variables & Properties
let pullRevolutions = 0;
let previousPullRevolutions = 0;
let sessionRevolutions = 0;

let eventAdvertisedIndex = 0;
let eventArray = [];

let isFirstSpinAfterRollChange = true;
let rollIsOnSpindle = null;
let accelerometerSettlingTime;
let realTimeClock;

// Timeouts and Intervals
let pullTimeout;
let sessionTimeout;
let checkRollStatusInterval;

// ------------------------------- Utility Functions -------------------------------
// See https://stackoverflow.com/a/9354310/1459580 for an explanation on this pattern
Object.defineProperty(String.prototype, "padTime", {
  value: function padTime() {
    return ("0" + this).substr(-2);
  },
  writable: true,
  configurable: true,
  enumerable: false,
});

// Formats a Date object into a YYMMDDhhmmss string for BLE advertising.
function formatDate(d) {
  // Get the last two digits of the year
  const year = d.getFullYear().toString().substr(-2);

  // Get the month, incrementing by 1 since it is 0 indexed
  const month = (d.getMonth() + 1).toString().padTime();

  const day = d.getDate().toString().padTime();
  const hours = d.getHours().toString().padTime();
  const minutes = d.getMinutes().toString().padTime();
  const seconds = d.getSeconds().toString().padTime();

  return year + month + day + hours + minutes + seconds;
}

// Rounds a number to the nearest quarter (0.25) and returns as float.
function roundToNearestQuarter(num) {
  return parseFloat((Math.round(num * 4) / 4).toFixed(2));
}

// ------------------------------- Event Handlers -------------------------------
// Handles the end of a pull event, processes revolutions, and updates session state.
function onPullEnd() {
  logger.info("pull timeout reached");
  clearTimeout(pullTimeout);

  logger.debug(`raw revs: ${pullRevolutions}`);
  const roundedRevolutions = roundToNearestQuarter(pullRevolutions);

  // Check for roll change noise
  // Note: The Math.abs is used since the revolutions count could be negative, indicating a change in expected roll pull direction.
  const isRollChangeNoise =
    isFirstSpinAfterRollChange && Math.abs(roundedRevolutions) <= REQUIRED_SPINS_AFTER_CHANGE;
  if (isRollChangeNoise) {
    logger.debug(`revs under roll change noise threshold. roundedRevs: ${roundedRevolutions}`);
  } else {
    logger.info(`saving pull to session. total revs: ${roundedRevolutions}`);

    // Note we are NOT taking the absolute value here.  This allows the roll to be rewound during a session.
    sessionRevolutions += roundedRevolutions;

    isFirstSpinAfterRollChange = false;
  }

  pullRevolutions = 0;
  previousPullRevolutions = 0;
}

// Handles the end of a session, logs a dispense event if needed, and resets counters.
function onSessionEnd() {
  logger.info(`session timeout reached. total session revolutions: ${sessionRevolutions}`);

  if (sessionRevolutions != 0) {
    logger.info("logging dispense event");
    const evnt = {
      type: eventType.dispense,
      date: getCurrentDateTimeFromRtc(),

      // taking absolute value here to remove effect of roll direction; assumes that more paper comes off the roll than is being rewound
      revs: Math.abs(sessionRevolutions),
    };
    eventArray.push(evnt);
    logger.debug(`dispense event:\n${JSON.stringify(evnt, null, 2)}`);
  }

  // // --- Store eventArray to flash ---
  // let log = [];
  // try {
  //   log = JSON.parse(Storage.read("events_log.json") || "[]");
  // } 
  // catch (e) {
  //   log = [];
  // }
  // log.push(eventArray); // or just the new session's events
  // Storage.write("events_log.json", JSON.stringify(log));
  // // ---

  // Reset counters
  sessionRevolutions = 0;
  pullRevolutions = 0;
  previousPullRevolutions = 0;

  sleep();
}

// Handles accelerometer events while the device is awake, tracks spindle movement and manages pull/session timeouts.
function onAccelWhileAwake(data) {
  logger.silly("accel event");

  // Any movement will keep the session going
  if (sessionTimeout) {
    clearTimeout(sessionTimeout);
  }

  sessionTimeout = setTimeout(onSessionEnd, SESSION_TIMEOUT_IN_MS);

  if (!rollIsOnSpindle) {
    logger.silly("ignoring pull revolutions while roll is off spindle");
    return;
  }

  // Determining the most recent revolutions
  pullRevolutions += data.gyro.z / PUCK_GYRO_CONSTANT;
  const spindleIsMoving = Math.abs(pullRevolutions - previousPullRevolutions) > PULL_REV_THRESHOLD;
  if (!spindleIsMoving) {
    logger.silly("rotation under pull threshold, ignoring");
    return;
  }

  logger.debug(`spindle is moving. revs: ${pullRevolutions}`);

  // Restart pull timeout since real movement was detected
  if (pullTimeout) {
    clearTimeout(pullTimeout);
  }

  pullTimeout = setTimeout(onPullEnd, PULL_TIMEOUT_IN_MS);
  previousPullRevolutions = pullRevolutions;
}

// Handles accelerometer events while the device is asleep, wakes device after settling time.
function onAccelWhileAsleep(_) {
  logger.debug("accel event");

  // ignore events while accelerometer settles
  if (getTime() < accelerometerSettlingTime) {
    logger.debug("ignoring accel event during settling period");
    return;
  }

  wake();
}

// ------------------------------- Main Functions -------------------------------
// Wakes the device, sets up accelerometer and roll status checks, and starts a session timeout.
function wake() {
  logger.info("waking up");

  Puck.removeAllListeners("accel");
  Puck.on("accel", onAccelWhileAwake);

  sessionTimeout = setTimeout(onSessionEnd, SESSION_TIMEOUT_IN_MS);
  checkRollStatusInterval = setInterval(checkRollStatus, CHECK_ROLL_STATUS_INTERVAL_TIME_IN_MS);
}

// Puts the device to sleep, disables accelerometer event handling, and stops roll status checks.
function sleep() {
  logger.info("going to sleep");
  accelerometerSettlingTime = getTime() + 2;

  Puck.removeAllListeners("accel");
  Puck.on("accel", onAccelWhileAsleep);

  clearInterval(checkRollStatusInterval);
}

// Reads the current date and time from the RTC and returns it as a Date object.
function getCurrentDateTimeFromRtc() {
  const dtstring = realTimeClock.readDateTime();
  const parts = dtstring.split(" ");
  const mdy = parts[0].split("/");
  const hms = parts[1].split(":");

  return new Date(2000 + parseInt(mdy[2]), parseInt(mdy[1]) - 1, mdy[0], hms[0], hms[1], hms[2]);
}

// Checks the IR sensor to determine if a roll is on the spindle, logs roll change events, and updates state.
function checkRollStatus() {
  logger.debug(`in checkRollStatus.  rollIsOnSpindle: ${rollIsOnSpindle}`);
  digitalWrite(D2, 1);

  // The pin check must occur at least one clock cycle after the pin write.  Hence the timeout.
  const afterPinWrite = function () {
    // Note: the IR detection hardware uses inverted logic.
    // If roll is off, the IR module does not detect reflected IR light and sets the read pin HIGH.
    // If roll is on, the IR module does detect reflected IR light and sets the read pin LOW.
    if (digitalRead(D1)) {
      logger.debug("roll is OFF");
      if (rollIsOnSpindle === true) {
        logger.info("roll was removed");

        // Removing the roll means the roll is either empty or the User will not use what is left.
        // As such, we clear out any existing pull revolutions and stop the pull.
        if (pullTimeout) {
          clearTimeout(pullTimeout);
        }

        pullRevolutions = 0;
        previousPullRevolutions = 0;
      }

      rollIsOnSpindle = false;
    } else {
      logger.debug("roll is ON");
      if (rollIsOnSpindle === false) {
        logger.info("roll was replaced");
        isFirstSpinAfterRollChange = true;

        // Clear the interval so that only one roll change detection occurs per session
        clearInterval(checkRollStatusInterval);

        logger.info("logging rollChange event");
        const evnt = {
          type: eventType.rollChange,
          date: getCurrentDateTimeFromRtc(),
        };
        eventArray.push(evnt);
        logger.debug(`rollChange event:\n${JSON.stringify(evnt, null, 2)}`);
      }

      rollIsOnSpindle = true;
    }

    digitalWrite(D2, 0);
  };
  setTimeout(afterPinWrite, 1);
}

// Prepares and advertises the latest event data over BLE using manufacturer data packets.
function advertiseEventData() {
  if (eventArray.length == 0) {
    logger.debug("no events to advertise");
    return;
  }

  let dataPacket = "";
  const evnt = eventArray[eventAdvertisedIndex];
  switch (evnt.type) {
    case eventType.dispense:
      dataPacket = `${formatDate(evnt.date)}|${evnt.revs}`;
      break;
    case eventType.rollChange:
      dataPacket = `${formatDate(evnt.date)}|C`;
      break;
    default:
      logger.error(`** Unknown eventType!  type: ${evnt.type}`);
      break;
  }

  eventAdvertisedIndex += 1;
  if (eventAdvertisedIndex == eventArray.length) {
    // Circle back to beginning of array
    eventAdvertisedIndex = 0;
  }

  if (dataPacket.length === 0) {
    return;
  }

  logger.info(`advertising: ${dataPacket}`);
  NRF.setAdvertising(
    {},
    {
      name: bleLocalName,
      manufacturer: KC_BLUETOOTH_COMPANY_ID,
      manufacturerData: dataPacket,
      showName: true,
      discoverable: true,
      connectable: false,
      scannable: true,
      interval: EVENT_DATA_BLE_ADVERTISING_INTERVAL_IN_MS,
    }
  );
}

// Removes events from the event array that are older than the expiration threshold.
function clearOldEvents() {
  logger.debug("clearing old events...");
  if (eventArray.length === 0) {
    logger.debug(`no events in array`);
    return;
  }

  let now = getCurrentDateTimeFromRtc();
  let threshold = new Date(now - EXPIRED_EVENT_AGE_IN_SEC * 1000);

  const originalArrayLength = eventArray.length;
  for (let i = eventArray.length - 1; i >= 0; i--) {
    logger.debug(`checking date ${i + 1} of ${originalArrayLength}`);
    if (eventArray[i].date < threshold) {
      logger.debug(`date ${i} is too old. deleting...`);
      eventArray.splice(i, 1);
    }
  }
}

// Initializes hardware peripherals (RTC, I2C, accelerometer) and sets up device state.
function initializeHardware() {
  digitalWrite(D31, 1);
  I2C1.setup({ scl: D29, sda: D30 });
  realTimeClock = DS3231.connect(I2C1, { DST: false });

  Accel.on();
  // scale to 2000dps
  Puck.accelWr(0x11, 0b00011100);
}

// Dumps flash memory event log to the console for debugging purposes.
function dumpEventLog() {
  let log = [];
  try {
    log = JSON.parse(Storage.read("events_log.json") || "[]");
  } catch (e) {
    print("Could not read or parse events_log.json");
    return;
  }
  log.forEach(function(session, idx) {
    print("Session " + (idx + 1) + ":");
    if (Array.isArray(session)) {
      session.forEach(function(evnt) {
        print("  " + JSON.stringify(evnt));
      });
    } else {
      print("  " + JSON.stringify(session));
    }
  });
}

// Dumps all events in eventArray to the terminal as JSON strings
function dumpEvents() {
  if (eventArray.length === 0) {
    print("No events recorded.");
    return;
  }
  eventArray.forEach(function(evnt, idx) {
    print("Event " + (idx + 1) + ": " + JSON.stringify(evnt));
  });
}

// ------------------------------- Serial Communication -------------------------------

// Set up Serial1 on D28 (RX) and D11 (TX) at 9600 baud
Serial1.setup(9600, { rx: D28, tx: D11 });

// Move the Espruino console to Serial1 so you can use the IDE or a terminal
E.setConsole(Serial1);

// Optional: Print a welcome message when serial is ready
Serial1.println("Serial interface ready. Type 'dumpEvents()' to see all events.");

// Optionally, handle incoming serial commands (simple command parser)
Serial1.on('data', function(data) {
  data = data.trim();
  if (data === "dump") {
    dumpEvents();
  } else if (data === "clear") {
    eventArray = [];
    Serial1.println("Event array cleared.");
  } else {
    Serial1.println("Unknown command: " + data);
    Serial1.println("Available: dump, clear");
  }
});


// ------------------------------- Startup -------------------------------
initializeHardware();

setInterval(clearOldEvents, CLEAR_OLD_EVENTS_INTERVAL_TIME_IN_MS);
setInterval(advertiseEventData, ADVERTISE_EVENT_DATA_INTERVAL_TIME_IN_MS);

wake();
