#include <Arduino.h>
#include "Si115X.h"

// Si115X::Si115X() {
// 	//empty constructor
// }

/**
 * Configures a channel at a given index
 */

void Si115X::config_channel(uint8_t index, uint8_t *conf){
    int len = sizeof(conf);
  
    if(len != 4 || index > 5)
      return;

    int inc = index * len;
    
    // ADCCONFIGx: 
    // - bits[7] - Reserved
    // - bits[6:5] - A/D conversion rate
    // - bits[4:0] - ADC MUX select (see below)
    //      00000 - Small IR   00001 - Medium IR   00010 - Large IR
    //      01011 - Visible    01101 - Large Visible
    param_set(Si115X::ADCCONFIG_0 + inc, conf[0]);

    // ADCSENSx:
    // - bits[7] - ADC high signal range enable
    // - bits[6:4] - an internal accumulation of samples
    // - bits[3:0] - Measurement time for 512 decimation rate
    param_set(Si115X::ADCSENS_0 + inc, conf[1]);

    // ADCPOSTx:
    // - bits[7] - Reserved
    // - bits[6] - HOSTOUTx size(0 = 16 bits, 1 = 24 bits)
    // - bits[5:3] - Number of bits to shift right of the output
    // - bits[2] - Threshold polarity
    // - bits[1:0] - Threshold enable
    param_set(Si115X::ADCPOST_0 + inc, conf[2]);

    // MEASCONFIGx:
    // - bits[7:6] - MEASCOUNTx select
    // - bits[5:4] - Reserved
    // - bits[3] - LEDx_A or LEDx_B BANK select
    // - bits[2:0] - LEDx enable
    param_set(Si115X::MEASCONFIG_0 + inc, conf[3]);   
}

/**
 * Writes data over i2c
 */
void Si115X::write_data(uint8_t addr, uint8_t *data, size_t len){
    Wire.beginTransmission(addr);
    Wire.write(data, len);
    Wire.endTransmission();
}

/**
 * Reads data from a register over i2c
 */
int Si115X::read_register(uint8_t addr, uint8_t reg, int bytesOfData){
    int val = -1;
  
    Si115X::write_data(addr, &reg, sizeof(reg));
    Wire.requestFrom(addr, bytesOfData);
  
    if(Wire.available())
      val = Wire.read();
	
    return val;
}

/**
 * param set as shown in the datasheet
 */
void Si115X::param_set(uint8_t loc, uint8_t val){
    uint8_t packet[2];
    int r;
    int cmmnd_ctr;

    do {
        cmmnd_ctr = Si115X::read_register(Si115X::DEVICE_ADDRESS, Si115X::RESPONSE_0, 1);
      
        packet[0] = Si115X::HOSTIN_0;
        packet[1] = val;
        Si115X::write_data(Si115X::DEVICE_ADDRESS, packet, sizeof(packet));
      
        packet[0] = Si115X::COMMAND;
        packet[1] = loc | (0B10 << 6);
        Si115X::write_data(Si115X::DEVICE_ADDRESS, packet, sizeof(packet));
      
        r = Si115X::read_register(Si115X::DEVICE_ADDRESS, Si115X::RESPONSE_0, 1);	    
    } while(r > cmmnd_ctr); 
}

/**
 * param query as shown in the datasheet
 */
int Si115X::param_query(uint8_t loc){
    int result = -1;
    uint8_t packet[2];
    int r;
    int cmmnd_ctr;

    do {
        cmmnd_ctr = Si115X::read_register(Si115X::DEVICE_ADDRESS, Si115X::RESPONSE_0, 1);
	    
        packet[0] = Si115X::COMMAND;
        packet[1] = loc | (0B01 << 6);
	    
        Si115X::write_data(Si115X::DEVICE_ADDRESS, packet, sizeof(packet));
	    
        r = Si115X::read_register(Si115X::DEVICE_ADDRESS, Si115X::RESPONSE_0, 1);
    } while(r > cmmnd_ctr);
	
    result = Si115X::read_register(Si115X::DEVICE_ADDRESS, Si115X::RESPONSE_1, 1);
	
    return result;
}

/**
 * Sends command to the command register
 */
void Si115X::send_command(uint8_t code){
    uint8_t packet[2];
    int r;
    int cmmnd_ctr;
    do {
        cmmnd_ctr = Si115X::read_register(Si115X::DEVICE_ADDRESS, Si115X::RESPONSE_0, 1);
	    
        packet[0] = Si115X::COMMAND;
        packet[1] = code;
	    
        Si115X::write_data(Si115X::DEVICE_ADDRESS, packet, sizeof(packet));
	    
        r = Si115X::read_register(Si115X::DEVICE_ADDRESS, Si115X::RESPONSE_0, 1); 
    } while(r > cmmnd_ctr);
}

/**
 * Returns int given a byte array
 */
int Si115X::get_int_from_bytes(uint8_t *data, size_t len){
    int result = 0;
    int shift = 8 * len;
	
    for(size_t i = 0; i < len; i++){
        shift -= 8;
        result |= ((data[i] << shift) & (0xFF << shift));
    }
	
    return result;
}




bool Si115X::Begin(bool mode){
    is_autonomous = mode;
    Wire.begin();
    // Wire.setClock(400000);
    // send_command(RESET_SW);
    if (ReadByte(0x00) != 0x51) {
        return false;
    }

    // Enable 2 channels for proximity measurement
    param_set(CHAN_LIST, 0B000011);
    // Enable Interrupt
    write_register(DEVICE_ADDRESS, IRQ_ENABLE, 0B000011);
    // Initialize LED current
    param_set(LED1_A, 0x3F);
    param_set(LED1_B, 0x3F);

    // Configure ADC and enable LED drive
    if (is_autonomous) {
        param_set(MEASRATE_H, 0);
        param_set(MEASRATE_L, 1);  // 1 for a base period of 800 us
        param_set(MEASCOUNT_0, 1); 
        param_set(MEASCOUNT_1, 1);
        param_set(THRESHOLD0_L, 0);
        param_set(THRESHOLD0_H, 0);
        uint8_t conf[4]; // ADCCONFIGx, ADCSENSx, ADCPOSTx, MEASCONFIGx
        conf[0] = 0B01100000; // 1x Small IR
        conf[1] = 0B00000010; // 48.8us Nominal Measurement time for 512 decimation rate
        conf[2] = 0B00000001; // 16-bits output, Interrupt when the measurement is larger than THRESHOLD0
        conf[3] = 0B01000001; // enable LED1A, the time between measurements is 800*MEASRATE*MEASCOUNT0 us
        config_channel(0, conf);
        conf[0] = 0B01101101; // 1x Visible
        conf[1] = 0B00000010; // 48.8us Nominal Measurement time for 512 decimation rate
        conf[2] = 0B00000001; // 16-bits output, Interrupt when the measurement is larger than THRESHOLD0
        conf[3] = 0B10001001; // enable LED1B, the time between measurements is 800*MEASRATE*MEASCOUNT1 us
        config_channel(1, conf);
        send_command(START);
    }
    else {
        param_set(ADCCONFIG_0, 0B01100000);
        param_set(MEASCONFIG_0, 0x00);
        param_set(ADCPOST_0, 0x00);
        param_set(ADCCONFIG_1, 0B01101101);
        param_set(MEASCONFIG_1, 0x00);
        param_set(ADCPOST_1, 0x00);
    }

    return true;

}

uint16_t Si115X::ReadIR(void) {
    if (!is_autonomous) send_command(FORCE);
    uint8_t data[2];
    data[0] = read_register(DEVICE_ADDRESS, HOSTOUT_0);
    data[1] = read_register(DEVICE_ADDRESS, HOSTOUT_1);
    return (data[0] << 8) + data[1];
}

uint16_t Si115X::ReadVisible(void) {
    if (!is_autonomous) send_command(FORCE);
    uint8_t data[2];
    data[0] = read_register(DEVICE_ADDRESS, HOSTOUT_2);
    data[1] = read_register(DEVICE_ADDRESS, HOSTOUT_3);
    return (data[0] << 8) + data[1];
}

uint8_t Si115X::ReadByte(uint8_t Reg) {
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write(Reg);
    Wire.endTransmission();
    Wire.requestFrom(DEVICE_ADDRESS, 1);
    return Wire.read();
}
