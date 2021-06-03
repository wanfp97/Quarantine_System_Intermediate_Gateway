#include "MyLora_E22.h"

bool MyLoRa_E22 :: set_e22_configuration(uint8_t addr_h, uint8_t addr_l, byte chan, bool lbt, TRANSMISSION_POWER dBm, 
AIR_DATA_RATE air_data_rate, uint8_t crypt_h, uint8_t crypt_l, bool keep_others) {
  ResponseStructContainer c;
  c = getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration*) c.data;
  
  configuration.ADDL = addr_l;
  configuration.ADDH = addr_h;
  configuration.CHAN = chan;
  configuration.OPTION.transmissionPower = dBm;
  configuration.SPED.airDataRate = air_data_rate;
  configuration.CRYPT.CRYPT_H = crypt_h;
  configuration.CRYPT.CRYPT_L = crypt_l;

  if(lbt) {
    configuration.TRANSMISSION_MODE.enableLBT = LBT_ENABLED;
  }
  else {
    configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  }

  if(!keep_others) {
    configuration.NETID = 0x00;
    configuration.SPED.uartBaudRate = UART_BPS_9600;
    configuration.SPED.uartParity = MODE_00_8N1;

    configuration.OPTION.subPacketSetting = SPS_240_00;
    configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
    

    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    configuration.TRANSMISSION_MODE.enableRepeater = REPEATER_DISABLED;
    
    
    configuration.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
    configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;
  }

  ResponseStatus rs = setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
	if(rs.code == SUCCESS) {
    return true;
  }
  else {
    return false;
  }
}