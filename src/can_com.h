/*
 * LOCO-CAN can class header file
 * 
 * @author: Thomas H Winkler
 * @copyright: 2020
 * @lizence: GG0
 * 
 * using can library:
 * https://github.com/sandeepmistry/arduino-CAN
 * https://github.com/ricaun/ArduinoUniqueID
 * https://github.com/SukkoPera/Arduino-Rokkit-Hash
 */

#ifndef CAN_COM_H
#define CAN_COM_H

#include <CAN.h>

#include <intelliTimeout.h>
#include "intelliLed.h"


/*
 * CAN speeds
 * 
 * 1000 bps = 1000E3
 * 500 bps = 500E3
 * ...
 * 
 * valid speed settings
 * 1000, 500, 250, 200, 125, 100, 80, 50, 40, 20, 10 ,5
 */


#define CAN_MAX_FILTER 8
#define CAN_ALIVE_TIMEOUT 500


struct CAN_MESSAGE {
    uint32_t id;
    uint16_t uuid;
    uint8_t data[8];
    uint8_t size;
};



class CAN_COM {

  public:
    CAN_COM(void); // construct with default CS and INT port
    CAN_COM(uint8_t CS, uint8_t INT); // user CS and INT ports

    bool begin(long speed, uint8_t led_port); // start communication with speed setting and status LED
    bool begin(long speed, uint8_t led_port1, uint8_t led_port2); // start communication with speed setting and r/w status LEDs

    bool alive(void);
    void set_alive(uint16_t alive_timeout); // set alive timeout

    bool send(byte* data, byte length, uint32_t id); // send data
    
    bool clear_filter();
    bool register_filter(uint16_t mask, uint16_t filter); // add mask and filter
    uint16_t read(CAN_MESSAGE* message); // receive data > true if no filter or filter match

    long uuid(void);

  private:

    void create_uuid(void);
    bool _begin(long speed);
    long _uuid;

    uint8_t _cs;
    uint8_t _int;
    
    uint16_t _masks[CAN_MAX_FILTER];
    uint16_t _filters[CAN_MAX_FILTER];

    uint8_t _filter_count;

    INTELLITIMEOUT _alive_timeout;
    bool _alive;

    INTELLILED _led_r;
    INTELLILED _led_w;
};

#endif
