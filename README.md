## EME - Egypt Makes Electronics
## SEITech Solutions

### Team Members
- Hossam Elwahsh - [![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=flat&logo=github&logoColor=white)](https://github.com/HossamElwahsh) [![LinkedIn](https://img.shields.io/badge/linkedin-%230077B5.svg?style=flat&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/hossam-elwahsh/)
- Ahmed Sakr - [![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=flat&logo=github&logoColor=white)](https://github.com/Ahmeddsakrrr)
- Ahmed Hisham - [![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=flat&logo=github&logoColor=white)](https://github.com/ahmedhish)
- Nada Salloum - [![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=flat&logo=github&logoColor=white)](https://github.com/nadasalloum)
- Salma Faragalla - [![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=flat&logo=github&logoColor=white)](https://github.com/SalmaFaragalla)
- Norhan Mohamed - [![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=flat&logo=github&logoColor=white)](https://github.com/NorhanMohamwd) [![LinkedIn](https://img.shields.io/badge/linkedin-%230077B5.svg?style=flat&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/norhan-mohamed-60b414213)

### CAN ESP Car Clone: Emulating Real Car Actions
- Tapping into the high speed can bus of a (Fiat Tipo 2019), clone, process and sends certain car actions to an RC car for it to emulate, 
- Actions including: (throttle position, steering wheel position, current transmission selection, selection of car lighting).
- Utilizing 2x STM32 microcontrollers running FreeRTOS along with two ESP8266 chips communicating over ESP-NOW protocol to minimize delay, one of which has an active webserver introducing a quick monitor system that can be accessed by connecting to a certain WIFI SSID with the correct password and a static IP address to access the monitoring webpage showing RC car status in real-time.

### Dist Files

| ECU | APP                       | MC          | OS                      | Description                                                                                                                                                                                                                                  | Build File                                 |
|-----|---------------------------|-------------|-------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------|
| 1   | HSPD CAN Processor        | STMF103C8T6 | FreeRTOS Kernel V10.0.1 | Handles monitoring, analyzing and processing of a real car high speed can bus messages, mapping and queueing certain actions to be sent over to ESP8266 using UART which will later send the queued actions via ESP-NOW to the RC Car System | [hex](dist/hspd_can_processor.hex)         |
| 2   | Data Transfer             | ESP8266     | -                       |                                                                                                                                                                                                                                              | [ino](dist/sender.ino)                     |
| 3   | Data Transfer + Webserver | ESP8266     | -                       |                                                                                                                                                                                                                                              | [ino](dist/rc_receiver_plus_webserver.ino) |
| 4   | RC Car Controller         | STMF103C8T6 | FreeRTOS Kernel V10.0.1 |                                                                                                                                                                                                                                              | uploading                                  |


---

### General Operational Notes

---

### General Issues [Resolved]

---

### Future Improvements
