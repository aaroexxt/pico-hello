// #include "LittleFS.h"

// void setup()
// {
//     delay(5000);
//     Serial.begin(115200);
//     Serial.println("LittleFS demo");

//     LittleFSConfig cfg;
//     cfg.setAutoFormat(false);
//     LittleFS.setConfig(cfg);

//     Serial.println("Initializing filesystem");
//     if (LittleFS.begin())
//     {
//         Serial.println("FS mount OK");
//     }
//     else
//     {
//         Serial.println("FS mount error");
//         while (1)
//         {
//         }
//     }

//     File test = LittleFS.open("/test.json", "r");
//     if (!test)
//     {
//         Serial.println("File open failed");
//         while (1)
//         {
//         }
//     }
//     uint8_t readLen = 5;
//     char *buf[readLen];
//     test.readBytes(*buf, readLen);
//     for (uint8_t i = 0; i < readLen; i++)
//     {
//         Serial.print(buf[i]);
//     }
// }

// void loop() {}