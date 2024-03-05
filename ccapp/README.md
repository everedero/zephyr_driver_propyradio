# Example application for CC2500 driver

## Description

This application tries to copy a third party remote control.

This example app can be run on the following boards:

* STM32 Nucleo F756ZX (nucleo\_f756zg)

For each board, an example overlay layer is provided.

```
BOARD="nucleo_f756zg";west build -b $BOARD -p always ccapp -- -DOVERLAY_CONFIG=bob.conf
```

## Pinout

| Name      | NRF24 pin | Nucleo pin | Nucleo logical GPIO | NRF52DK pin | NRF52 logical GPIO | ESP32WROOM pin | ESP32 logical GPIO |
|-----------|-----------|------------|---------------------|-------------|--------------------|----------------|--------------------|
| GND       | 1         | GND        |                     | GND         |                    | GND            |                    |
| VCC       | 2         | 3V3        |                     | VDD         |                    | 3V3            |                    |
| CSN       | 4         | D24        | PA4                 | P0.22       | D10                | D3             | 10                 |
| SCK       | 5         | D23        | PB3                 | P0.25       | D13                | 14             | 14                 |
| MOSI      | 6         | D22        | PB5                 | P0.23       | D11                | 15             | 15                 |
| MISO      | 7         | D25        | PB4                 | P0.24       | D12                | 2              | 2                  |

### Pinout diagrams
#### CC2400
<!--

VCC   | 1
GND   | 2
SI    | 3
SCLK  | 3
SO    | 4
GDO2  | 5
GDO0  | 6
CSN   | 7
PA    | 8
LNA   | 9

-->
![Kroki generated
PlantUML](https://kroki.io/ditaa/svg/eNpTUIACbV1dXTDmAvHc_VwUFGoUDIHYCIjDnJ25FFBUAukasJCzK4ipYAwkTYC0c7CfgoatX3CwJkJDDYwGCwU7e4OETIFMMyDt6x_siUupr2ewP1DIHMi0ANKeQYG4VMIdBgLaAAqjJlw=)


#### STM32 Nucleo
<!--
                                                                         CN7

                                                                       +-------+
                                                                   D16 | 1 | 2 | D15
                                                                       |   |   |
                                                                   D17 | 3 | 4 | D14
                                                                       |   |   |
                                                                   D18 | 5 | 6 |
                    CN8                                                |   |   |
                                                                   D19 | 7 | 8 |
                  +-------+                                            |   |   |
               NC | 1 | 2 | D43                                    D20 | 9 | 10| D13
                  |   |   |                                            |   |   |
                  | 3 | 4 | D44                                    D21 | 11| 12| D12
                  |   |   |                                            |   |   |
                  | 5 | 6 | D45                    (6) SPI1_MOSI   D22 | 13| 14| D11
                  |   |   |                                            |   |   |
              3V3 | 7 | 8 | D46                    (5) SPI1_SCK    D23 | 15| 16| D10
                  |   |   |                                            |   |   |
               5V | 9 | 10| D47                    (4) SPI1_NSS    D24 | 17| 18| D9
                  |   |   |                                            |   |   |
              GND | 11| 12| D48                    (7) SPI1_MISO   D25 | 19| 20| D8
                  |   |   |                                            |   |   |
              GND | 13| 14| D49                                        +-------+
                  |   |   |
                  | 15| 16| D50                                        +-------+
                  |   |   |                                            | 1 | 2 | D7
                  +-------+                                            |   |   |
                                                                       | 3 | 4 | D6
                  +-------+                                            |   |   |
              A0  | 1 | 2 | D51                                   GND  | 5 | 6 | D5
                  |   |   |                                            |   |   |
              A1  | 3 | 4 | D52                                    A6  | 7 | 8 | D4
                  |   |   |                                            |   |   |
              A2  | 5 | 6 | D53                                    A7  | 9 | 10| D3
                  |   |   |                                            |   |   |
              A3  | 7 | 8 | D54                                    A8  | 11| 12| D2
                  |   |   |                                            |   |   |
              A4  | 9 | 10| D55                                    D26 | 13| 14| D1
                  |   |   |                                            |   |   |
              A5  | 11| 12| GND                                    D27 | 15| 16| D0
                  |   |   |                                            |   |   |
              D72 | 13| 14| D56                                    GND | 17| 18| D42
                  |   |   |                                            |   |   |
              D71 | 15| 16| D57                                    D28 | 19| 20| D41
                  |   |   |                                            |   |   |
              D70 | 17| 18| D58                                    D29 | 21| 22| GND
                  |   |   |                                            |   |   |
              D69 | 19| 20| D59                                    D30 | 23| 24| D40
                  |   |   |                                            |   |   |
              D68 | 21| 22| D60                                    D31 | 25| 26| D39
                  |   |   |                                            |   |   |
              GND | 23| 24| D61                                    GND | 27| 28| D38
                  |   |   |                                            |   |   |
              D67 | 25| 26| D62                                    D32 | 29| 30| D37
                  |   |   |                                            |   |   |
              D66 | 27| 28| D63                                    D33 | 31| 32| D36
                  |   |   |                                            |   |   |
              D65 | 29| 30| D64                                    D34 | 33| 34| D35
                  |   |   |                                            |   |   |
                  +-------+                                            +-------+

                  CN9                                                  CN10
-->
![Kroki generated
PlantUML](https://kroki.io/ditaa/svg/eNrNmMFO5DAMhu_zFDkOQkhxHDvpsWoXNEJbdqnElRfh4dduZ0oiFRRQMksl3xr36z9__KdjTKVrmMLhUKnX7d163dZoOAKbNwNSTmoEqkX5dqk6lEGaoZRfKP0PpYzSjKT4g4bDFP8zYSfNVMu423CzVhXCaUit5bGI0Fm5WSnB6i-Nh08eWEvHxFrel1HqawFIOaV016E8W0soaW_xkW_M_OcEr7-f5tNCqcIDSnmlhNaU-ILv9hJK3qWkM-U8PK5a6iIgKVZKK8Z5vn89Pf81x3jTXFh6Sf3mwy6yPyNP87wiq1cgSEVZ1JkVefglt2Jz5IdpTM3nd0fKMVy8cJqfFmQ1D3SyGfU945UgL97zXY10-3x3bBYiW_VhX5Njm3eh-Xz9fuht445bQ_Y2E4WgoJd6J5121NqsPWSikCtp1rPJhl1zSJeLUhSnfTDpeGuepj1molBRmvbRpAOteZj2PhOFqCzyOQvT5pCUirJsiRLIkA5C2xpyDNkJg9gUbu8kPb1rTwlZPIQyKWMamB7aU9pUFopllGpjJy5xq0uaU3KXykJFuT6ivpoTm7jlMNDelxwTWUa2ZZRLTIlNnNoEu-uckTZZGMp3jxObOLUJxvZahlQWdmVa6mBwYhNccie0p-RUFi772ETNfBSboNoEuT0lpbJw2ccm6qkExSaoNkG6xsfmtw6B78fpw95_H5358jVMYP8BcKUNlA==)


#### NRF52DK
<!--

                                                            +----+
                                                    P0.27   | 10 |
                                                            |    |
                                                    P0.28   | 9  |
                                                            |    |
                                                    P0.02   | 8  |
        o                                                   |    |
     +---+                                          GND     | 7  |
     | 1 | VDD                                              |    |
     |   |                                (5) SCK   P0.25   | 6  |
     | 2 | VDD                                              |    |
     |   |                                (7) MISO  P0.24   | 5  |
     | 3 | RESET                                            |    |
     |   |                                (6) MOSI  P0.23   | 4  |
     | 4 | VDD                                              |    |
     |   |                                (4) CSN   P0.22   | 3  |
     | 5 | 5V                                               |    |
     |   |                                          P0.20   | 2  |
     | 6 | GND                                              |    |
     |   |                                          P0.19   | 1  |
     | 7 | GND                                              |    |
     |   |                                                  +----+
     | 8 | NC                                                 o
     |   |                                                   P4
     +---+
      P1                                                    +----+
        o                                           P0.18   | 8  |
     +---+                                                  |    |
     | 1 | P0.03                                    P0.17   | 7  |
     |   |                                                  |    |
     | 2 | P0.04                                    P0.16   | 6  |
     |   |                                                  |    |
     | 3 | P0.28                                    P0.15   | 5  |
     |   |                                                  |    |
     | 4 | P0.29                                    P0.14   | 4  |
     |   |                                                  |    |
     | 5 | P0.30                                    P0.13   | 3  |
     |   |                                                  |    |
     | 6 | P0.31                                    P0.12   | 2  |
     |   |                                                  |    |
     +---+                                          P0.11   | 1  |
      P2                                                    |    |
                                                            +----+
                                                              o
                                                             P3

                                                            +----+
                                                    P0.10   | 9  |
                                                            |    |
                                                    P0.09   | 8  |
                                                            |    |
                                                    P0.08   | 7  |
                                                            |    |
                                                    P0.07   | 6  |
                                                            |    |
                                                    P0.06   | 5  |
                                                            |    |
                                                    P0.05   | 4  |
                                                            |    |
                                                    P0.21   | 3  |
                                                            |    |
                                                    P0.01   | 2  |
                                                            |    |
                                                    P0.00   | 1  |
                                                            |    |
                                                            +----+
                                                              o
                                                             P6
-->
![Kroki generated
PlantUML](https://kroki.io/ditaa/svg/eNrNl81uhCAURvc-Bcsa04Z_dK2mMU0dOzbzGr6AD19AplpoJqgoQ4ITF_fLkStHJgE7RvYqR5ZsKe3gGxbydwQIgjHZQzHqy2aKXEcUICoFxDoiX0YMOylUdzL_0ve2MhHiN0J2R85bVW2nGO_3j8YLS0FffpiOMF3CFxH4HAqRgs-mv0wUVJewRQSR81r39fexFFxSXPpmoiC6hC4i6DlrQVNQ9q3pCDbPP0cwNW_gaIo8Bc31y1BA8y7MEVzO-3t7IAWRa1GbnYoKsy_mCHEKxWP9KneMoC1Xhwy7EEBHF7Ix6upQgI_JsE6gKLcEutJ-_7dF2U_ZmfhSCEegm5b1LwU2FNSXgjsCDUBBJgr9vfSiYI5AA1BQQ1H4UlBHoAEo2ERBoC8FcQQagIIbCuRLgR2B7qZYuc0UBbIECjp88oErwPnVEejW0ZEk1jOoZsCnOP0W7uk3AkVuyTsOhbDkHYeCW_KOQ8Esecf5h4gsecdZC2TJOw4FtOV9OsVTyZv_APQBfRw=)

### ESP32-WROOM
<!--
            +-----+       +------------------+       +-----+
            | 3V3 |       |                  |       | GND |
            |     |       |                  |       |     |
            | EN  |       |                  |       | 23  |
            |     |       |                  |       |     |
            | VP  |       |    ESPRESSIF     |       | 22  |
            |     |       |                  |       |     |
            | VN  |       |    ESP32-WROOM   |       | TX  |
            |     |       |                  |       |     |
            | 34  |       |                  |       | RX  |
            |     |       |                  |       |     |
            | 35  |       |                  |       | 21  |
            |     |       |                  |       |     |
            | 32  |       |                  |       | GND |
            |     |       |                  |       |     |
            | 33  |       +------------------+       | 19  |
            |     |                                  |     |
            | 25  |                                  | 18  |
            |     |                                  |     |
            | 26  |                                  | 5   |
            |     |                                  |     |
            | 27  |                                  | 17  |
            |     |                                  |     |
(5) SCK     | 14  |                                  | 16  |
            |     |                                  |     |
            | 12  |                                  | 4   |
            |     |                                  |     |
            | GND |                                  | 0   |
            |     |                                  |     |
            | 13  |                                  | 2   |  MISO (7)
            |     |                                  |     |
            | D2  |                                  | 15  |  MOSI (6)
            |     |                                  |     |
(4) CSN     | D3  |                                  | D1  |
            |     |                                  |     |
            | CMD |                                  | D0  |
            |     |                                  |     |
            | 5V  |                                  | CLK |
            |     |                                  |     |
            +-----+                                  +-----+
-->
![Kroki generated
PlantUML](https://kroki.io/ditaa/svg/eNrFls0KgzAMx-8-RY4VEdbGuu1c3RDnx-xwexgffptOXHVIwMByKJTk31-aNFCAyYLwbYGzc8x1Bd6XFjrAFl_ruFvY5DqXCXTe0kcS9-tMnJZEsULgJbf1LCK1dZNam53mZMVNLpdkVOG9qarCEd8ezGSMiOKGnaypfZbcZAV_etuIU8TKSHYgj-t3XrHfZKWJYnngJsdEsQZu8p565yEwa64gDv62FIT2wZp8PDmipjBUyaQAAn3OKkhFFEfc9e_HhyLecZMlEsVqOKLIbAViz1r3hFp3OYxmUdkMRLz1-UU-GFuOOVDrkEjmDpiC2Ptkx0zWLVFsLjkn2f1wrdgn8An4ooaa)
