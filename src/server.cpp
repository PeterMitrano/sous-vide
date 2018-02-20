#include <Arduino.h>

#include "server.h"
#include "sous_vide.h"

extern ESP8266WebServer server;

void handleRoot() {
  String html = "<html><head><meta http-equiv='refresh' content='5'><script src='https://cdn.plot.ly/plotly-latest.min.js'></script></head><body>";
  html += "<h1>Sous Vide</h1>";
  html += "<br>";
  html += "<br>";
  html += "Current Temperature (F): ";
	html += formatTemp(current_temp_g);
  html += "<br>";
  html += "Time Remaining: ";
	html += formatTime(time_left_sec_g);
  html += "<br>";
  html += "<div id='graph'></div>";
  html += "<br>";
  html += "(This page will update every 5 seconds)";
  html += "</body>";
  html += "<script>";
  html += "Plotly.plot('graph',[{";
  html += "y:[";
  for (auto t : temps) {
    html += t;
    html += ",";
  }
  html += "],mode:'lines+markers'}]);";
  html += "</script>";
  html += "</html>";
  server.send(200, "text/html", html);
}

void handleNotFound() {
  server.send(404, "text/html", "404. No way bruh.");
}
