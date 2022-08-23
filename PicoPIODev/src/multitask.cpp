// #include "Arduino.h"

// void setup()
// {
//   Serial.begin(115200);
//   delay(5000);
//   Serial.println("C0 started");
//   rp2040.fifo.push(0);
// }

// bool resetFlag = 0;
// void loop()
// {
//   if (rp2040.fifo.available())
//   {
//     uint32_t data = rp2040.fifo.pop();
//     Serial.printf("C0 received: %d\n", data);
//     if (resetFlag)
//     {
//       data = -1;
//       resetFlag = 0;
//     }
//     rp2040.fifo.push(data + 1);
//   }

//   if (BOOTSEL)
//   {
//     Serial.println("Bootsel pressed");
//     while (BOOTSEL)
//     {
//       delay(1);
//     }
//     resetFlag = 1;
//   }
// }

// // Running on core1
// void setup1()
// {
//   delay(5000);
//   Serial.println("C1 started");
// }

// void loop1()
// {
//   if (rp2040.fifo.available())
//   {
//     uint32_t data = rp2040.fifo.pop();
//     Serial.printf("C1 received: %d\n", data);
//     rp2040.fifo.push(data + 1);
//   }
// }