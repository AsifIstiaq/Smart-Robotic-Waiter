#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <WiFi.h>
#include <WebServer.h>
#include "config.h"
#include "navigation.h"

extern unsigned long lastStateChangeTime;

WebServer server(80);

const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Waiter Robot</title>
  <style>
    body { font-family: sans-serif; text-align: center; background:#f4f4f4; margin:0; padding:24px; }
    h1 { color:#333; }
    button {
      width: 80%; max-width: 320px; margin: 10px auto; display:block;
      padding: 18px; font-size: 20px; border: none; border-radius: 10px;
      background:#2c7be5; color:white; cursor:pointer;
    }
    button:active { background:#1a5cbf; }
    #status { margin-top: 20px; font-size: 16px; color:#444; }
  </style>
</head>
<body>
  <h1>Smart Waiter Robot</h1>
  <button onclick="go(1)">Table 1</button>
  <button onclick="go(2)">Table 2</button>
  <button onclick="go(3)">Table 3</button>
  <button onclick="go(4)">Home</button>
  <button style="background:#d32f2f;" onclick="stopRobot()">STOP</button>
  <div id="status">Loading status...</div>

  <script>
    function go(dest) {
      fetch('/goto?dest=' + dest)
        .then(r => r.json())
        .then(d => { document.getElementById('status').innerText = d.msg; })
        .catch(() => {});
    }

    function stopRobot() {
      fetch('/stop')
        .then(r => r.json())
        .then(d => {
        document.getElementById('status').innerText = d.msg;
    })
    .catch(() => {});
}

    function poll() {
      fetch('/status')
        .then(r => r.json())
        .then(d => {
          document.getElementById('status').innerText =
            'State: ' + d.state + ' | Location: ' + d.location + ' | Dest: ' + d.dest;
        })
        .catch(() => {});
    }

    setInterval(poll, 1000);
    poll();
  </script>
</body>
</html>
)HTML";

inline void handleRoot() {
  server.send_P(200, "text/html", CONTROL_PAGE);
}

inline void handleGoto() {
  if (!server.hasArg("dest")) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"missing dest\"}");
    return;
  }

  int destArg = server.arg("dest").toInt();

  if (destArg < 1 || destArg > 4) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"dest must be 1-4\"}");
    return;
  }

  Destination dest = static_cast<Destination>(destArg);
  bool started = startJourney(dest);

  if (started) {
    lastStateChangeTime = millis();
  }

  String json = "{\"ok\":";
  json += started ? "true" : "false";
  json += ",\"msg\":\"";
  json += started ? (String("Heading to ") + destName(dest))
                  : "Robot busy or invalid request (must return Home first)";
  json += "\"}";

  server.send(200, "application/json", json);
}

inline void handleStop() {
  emergencyStop = true;
  stopMotors();

  robotState = STATE_IDLE;
  activeDest = currentLoc;

  String json =
  "{\"ok\":true,\"msg\":\"Robot stopped immediately\"}";

  server.send(200, "application/json", json);
}

inline void handleStatus() {
  String json = "{\"state\":\"";
  json += stateName(robotState);
  json += "\",\"location\":\"";
  json += destName(currentLoc);
  json += "\",\"dest\":\"";
  json += destName(activeDest);
  json += "\"}";

  server.send(200, "application/json", json);
}

inline void webInterfaceInit() {
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);

  Serial.print(F("AP started. Connect to \""));
  Serial.print(WIFI_AP_SSID);
  Serial.println(F("\" and browse to 192.168.4.1"));
  Serial.print(F("AP IP address: "));
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/goto", handleGoto);
  server.on("/status", handleStatus);
  server.on("/stop", handleStop);
  server.begin();
}

inline void webInterfaceLoop() {
  server.handleClient();
}

#endif
