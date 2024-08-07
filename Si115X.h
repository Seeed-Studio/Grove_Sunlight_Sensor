#ifndef SI115X_H
#define SI115X_H

#include <Arduino.h>
#include <Wire.h>

class Si115X
{
	public:
		typedef enum {
			DEVICE_ADDRESS = 0x53			
		} UnitAddress;

		typedef enum {
			RESET_CMD_CTR = 0x00,
			RESET_SW = 0x01,
			FORCE = 0x11,
			PAUSE = 0x12,
			START = 0x13,
			PARAM_QUERY = 0x40,
			PARAM_SET = 0x80
		} CommandCodes;
		
		typedef enum {	
			PART_ID = 0x00,
			REV_ID = 0x01,
			MFR_ID = 0x02,
			INFO_0 = 0x03,
			INFO_1 = 0x04,
			HOSTIN_3 = 0x07,
			HOSTIN_2 = 0x08,
			HOSTIN_1 = 0x09,
			HOSTIN_0 = 0x0A,
			COMMAND = 0x0B,
			IRQ_ENABLE = 0x0F,
			RESPONSE_1 = 0x10,
			RESPONSE_0 = 0x11,
			IRQ_STATUS = 0x12,
			HOSTOUT_0 = 0x13,
			HOSTOUT_1 = 0x14,
			HOSTOUT_2 = 0x15,
			HOSTOUT_3 = 0x16,
			HOSTOUT_4 = 0x17,
			HOSTOUT_5 = 0x18,
			HOSTOUT_6 = 0x19,
			HOSTOUT_7 = 0x1A,
			HOSTOUT_8 = 0x1B,
			HOSTOUT_9 = 0x1C,
			HOSTOUT_10 = 0x1D,
			HOSTOUT_11 = 0x1E,
			HOSTOUT_12 = 0x1F,
			HOSTOUT_13 = 0x20,
			HOSTOUT_14 = 0x21,
			HOSTOUT_15 = 0x22,
			HOSTOUT_16 = 0x23,
			HOSTOUT_17 = 0x24,
			HOSTOUT_18 = 0x25,
			HOSTOUT_19 = 0x26,
			HOSTOUT_20 = 0x27,
			HOSTOUT_21 = 0x28,
			HOSTOUT_22 = 0x29,
			HOSTOUT_23 = 0x2A,
			HOSTOUT_24 = 0x2B,
			HOSTOUT_25 = 0x2C	
		} RegisterAddress;

		typedef enum {		
			I2C_ADDR = 0x00,
			CHAN_LIST = 0x01,

			ADCCONFIG_0 = 0x02,
			ADCSENS_0 = 0x03,
			ADCPOST_0 = 0x04,
			MEASCONFIG_0 = 0x05,
			
			ADCCONFIG_1 = 0x06,
			ADCPOST_1 = 0x08,
			ADCSENS_1 = 0x07,
			MEASCONFIG_1 = 0x09,

			ADCCONFIG_2 = 0x0A,
			ADCSENS_2 = 0x0B,
			ADCPOST_2 = 0x0C,
			MEASCONFIG_2 = 0x0D,

			ADCCONFIG_3 = 0x0E,
			ADCSENS_3 = 0x0F,
			ADCPOST_3 = 0x10,
			MEASCONFIG_3 = 0x11,

			ADCCONFIG_4 = 0x12,
			ADCSENS_4 = 0x13,
			ADCPOST_4 = 0x14,
			MEASCONFIG_4 = 0x15,

			ADCCONFIG_5 = 0x16,
			ADCSENS_5 = 0x17,
			ADCPOST_5 = 0x18,
			MEASCONFIG_5 = 0x19,
			
			MEASRATE_H = 0x1A,
			MEASRATE_L = 0x1B,
			MEASCOUNT_0 = 0x1C,
			MEASCOUNT_1 = 0x1D,
			MEASCOUNT_2 = 0x1E,
			
			LED1_A = 0x1F,
			LED1_B = 0x20,
			LED3_A = 0x21,
			LED3_B = 0x22,
			LED2_A = 0x23,
			LED2_B = 0x24,

			THRESHOLD0_H = 0x25,
			THRESHOLD0_L = 0x26,
			THRESHOLD1_H = 0x27,
			THRESHOLD1_L = 0x28,
			UPPER_THRESHOLD_H = 0x29,
			UPPER_THRESHOLD_L = 0x2A,
			BURST = 0x2B,
			LOWER_THRESHOLD_H = 0x2C,
			LOWER_THRESHOLD_L = 0x2D
		} ParameterAddress;
		
		Si115X(uint8_t addr = DEVICE_ADDRESS);
		void config_channel(uint8_t index, const uint8_t *conf);
		void write_data(uint8_t addr, const uint8_t *data, size_t len);
		int read_register(uint8_t addr, uint8_t reg, int bytesOfData);
		uint8_t read_register(uint8_t addr, uint8_t reg) {
			return read_register(addr, reg, 1);
		}
		void write_register(uint8_t addr, uint8_t reg, uint8_t val) {
			uint8_t data[2] = {reg, val};
			write_data(addr, data, 2);
		}

		void param_set(uint8_t loc, uint8_t val);
		int param_query(uint8_t loc);
		uint8_t send_command(uint8_t code);
		int get_int_from_bytes(const uint8_t *data, size_t len);

		bool Begin(bool mode);
		bool Begin(void) {
			return Begin(false);
		}
		uint16_t ReadIR(void);
		uint16_t ReadVisible(void);
		uint8_t ReadByte(uint8_t Reg);

	private:
		bool is_autonomous;
		uint8_t device_address;
};

#endif
