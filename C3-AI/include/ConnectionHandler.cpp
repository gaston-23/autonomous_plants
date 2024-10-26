// /* SECRET_ fields are in `arduino_secrets.h` (included below)
//  *
//  * If using a WiFi board (Arduino MKR1000, MKR WiFi 1010, Nano 33 IoT, UNO
//  * WiFi Rev 2 or ESP8266/32), create a WiFiConnectionHandler object by adding
//  * Network Name (SECRET_WIFI_SSID) and password (SECRET_WIFI_PASS) in the
//  * arduino_secrets.h file (or Secrets tab in Create Web Editor).
//  *
//  *    WiFiConnectionHandler conMan(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
//  *
//  * Manual configuration
//  *    EthernetConnectionHandler conMan(SECRET_IP, SECRET_DNS, SECRET_GATEWAY, SECRET_NETMASK);
//  *
//  * Manual configuration will fallback on DHCP mode if SECRET_IP is invalid or equal to INADDR_NONE.
//  *
//  */

// #include <Arduino_ConnectionHandler.h>

// #include "arduino_secrets.h"

// #define CONN_TOGGLE_MS 60000

// WiFiConnectionHandler conMan(SECRET_SSID, SECRET_OPTIONAL_PASS);

// bool attemptConnect = false;
// uint32_t lastConnToggleMs = 0;

// void setup() {
//   /* Initialize serial debug port and wait up to 5 seconds for port to open */
//   Serial.begin(9600);
//   for(unsigned long const serialBeginTime = millis(); !Serial && (millis() - serialBeginTime <= 5000); ) { }

// #ifndef __AVR__
//   /* Set the debug message level:
//    * - DBG_ERROR: Only show error messages
//    * - DBG_WARNING: Show warning and error messages
//    * - DBG_INFO: Show info, warning, and error messages
//    * - DBG_DEBUG: Show debug, info, warning, and error messages
//    * - DBG_VERBOSE: Show all messages
//    */
//   setDebugMessageLevel(DBG_INFO);
// #endif

//   /* Add callbacks to the ConnectionHandler object to get notified of network
//    * connection events. */
//   conMan.addCallback(NetworkConnectionEvent::CONNECTED, onNetworkConnect);
//   conMan.addCallback(NetworkConnectionEvent::DISCONNECTED, onNetworkDisconnect);
//   conMan.addCallback(NetworkConnectionEvent::ERROR, onNetworkError);

//   Serial.print("Network Adapter Interface: ");
//   switch (conMan.getInterface()) {
//     case NetworkAdapter::WIFI:
//       Serial.println("Wi-Fi");
//       break;
//     case NetworkAdapter::ETHERNET:
//       Serial.println("Ethernet");
//       break;
//     case NetworkAdapter::NB:
//       Serial.println("Narrowband");
//       break;
//     case NetworkAdapter::GSM:
//       Serial.println("GSM");
//       break;
//     case NetworkAdapter::LORA:
//       Serial.println("LoRa");
//       break;
//     case NetworkAdapter::CATM1:
//       Serial.println("Category M1");
//       break;
//     case NetworkAdapter::CELL:
//       Serial.println("Cellular");
//       break;
//     default:
//       Serial.println("Unknown");
//       break;
//   }
// }

// void loop() {
//   /* Toggle the connection every `CONN_TOGGLE_MS` milliseconds */
//   if ((millis() - lastConnToggleMs) > CONN_TOGGLE_MS) {
//     Serial.println("Toggling connection...");
//     if (attemptConnect) {
//       conMan.connect();
//     } else {
//       conMan.disconnect();
//     }
//     attemptConnect = !attemptConnect;
//     lastConnToggleMs = millis();
//   }

//   /* The following code keeps on running connection workflows on our
//    * ConnectionHandler object, hence allowing reconnection in case of failure
//    * and notification of connect/disconnect event if enabled (see
//    * addConnectCallback/addDisconnectCallback) NOTE: any use of delay() within
//    * the loop or methods called from it will delay the execution of .update(),
//    * which might not guarantee the correct functioning of the ConnectionHandler
//    * object.
//    */
//   conMan.check();
// }

// void onNetworkConnect() {
//   Serial.println(">>>> CONNECTED to network");
// }

// void onNetworkDisconnect() {
//   Serial.println(">>>> DISCONNECTED from network");
// }

// void onNetworkError() {
//   Serial.println(">>>> ERROR");
// }