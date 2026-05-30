#pragma once

// DEBUG_SERIAL=0 → ROS ile çalışırken (aynı port, binary protokol bozulmasın)
// DEBUG_SERIAL=1 → Serial monitörle standalone test

#ifndef DEBUG_SERIAL
  #define DEBUG_SERIAL 0
#endif

#if DEBUG_SERIAL
  #define DBG_PRINTF(fmt, ...)   Serial.printf(fmt, ##__VA_ARGS__)
  #define DBG_PRINTLN(x)         Serial.println(x)
  #define DBG_PRINT(x)           Serial.print(x)
#else
  #define DBG_PRINTF(fmt, ...)   ((void)0)
  #define DBG_PRINTLN(x)         ((void)0)
  #define DBG_PRINT(x)           ((void)0)
#endif
