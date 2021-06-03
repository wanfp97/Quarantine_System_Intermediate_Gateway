#ifndef MY_LORAE22_H
#define MY_LORAE22_H

#include "LoRa_E22.h"
class MyLoRa_E22 : public LoRa_E22 {
    public :
        using LoRa_E22 :: LoRa_E22;
        bool set_e22_configuration(uint8_t addr_h, uint8_t addr_l, byte CHAN, bool LBT, TRANSMISSION_POWER dBm, AIR_DATA_RATE air_data_rate, uint8_t crypt_h, uint8_t crypt_l, bool keep_others);
};

#endif