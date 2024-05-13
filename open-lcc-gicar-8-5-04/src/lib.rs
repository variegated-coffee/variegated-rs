#![no_std]

use bitflags::bitflags;

struct SensorMessage {
    cn2: bool,
    cn7: bool,
    cn4_adc_low_gain: Varint,
    cn4_adc_high_gain: Varint,
    cn3_adc_low_gain: Varint,
    cn3_adc_high_gain: Varint,
    cn1_capacitance: Varint,
}

enum SensorMessageError {
    WrongHeader,
    InvalidChecksum
}

impl SensorMessage {
    fn to_bytes(&self) -> [u8; 18] {
        let mut bytes: [u8; 18] = [0; 18];
        bytes[0] = 0x81;

        if self.cn2 {
            bytes[1] |= 0x40;
        }

        if self.cn7 {
            bytes[1] |= 0x02;
        }

        bytes[2..5].copy_from_slice(&self.cn4_adc_low_gain.bytes);
        bytes[5..8].copy_from_slice(&self.cn4_adc_high_gain.bytes);
        bytes[8..11].copy_from_slice(&self.cn3_adc_low_gain.bytes);
        bytes[11..14].copy_from_slice(&self.cn3_adc_high_gain.bytes);
        bytes[14..17].copy_from_slice(&self.cn3_adc_high_gain.bytes);
        bytes[17] = calculate_checksum(&bytes[1..17], 0x01);
        
        bytes
    }
    
    fn from_bytes(bytes: [u8; 18]) -> Result<Self, SensorMessageError> {
        if bytes[0] != 0x81 {
            return Err(SensorMessageError::WrongHeader);
        }

        if calculate_checksum(&bytes[1..17], 0x01) != bytes[17] {
            return Err(SensorMessageError::InvalidChecksum);
        }

        Ok(SensorMessage {
            cn2: bytes[1] & 0x40 != 0,
            cn7: bytes[1] & 0x02 != 0,
            cn4_adc_low_gain: Varint::new([bytes[2], bytes[3], bytes[4]]),
            cn4_adc_high_gain: Varint::new([bytes[5], bytes[6], bytes[7]]),
            cn3_adc_low_gain: Varint::new([bytes[8], bytes[9], bytes[10]]),
            cn3_adc_high_gain: Varint::new([bytes[11], bytes[12], bytes[13]]),
            cn1_capacitance: Varint::new([bytes[14], bytes[15], bytes[16]]),
        })
    }
}

bitflags! {
    struct GicarShiftRegister2Flags: u8 {
        const CN5 = 1 << 0;
        const FA10 = 1 << 4;
        const FA9 = 1 << 5;
    }
}

bitflags! {
    struct GicarShiftRegister1Flags: u8 {
        const CN6_1 = 1 << 0;
        const CN6_3 = 1 << 1;
        const CN6_5 = 1 << 2;
        const CN9 = 1 << 3;
        const FA7 = 1 << 4;
        const FA8 = 1 << 5;
        const CN10_DISABLE_OLED_12V = 1 << 6; // Maybe. They are connected to some kind of transistor that would presumably cut power to 12V on CN10 if used, but this has not been tested yet
        const CN10_DISABLE_OLED_3V3 = 1 << 7; // Maybe. They are connected to some kind of transistor that would presumably cut power to 3V3 OLED on CN10 if used, but this has not been tested yet
    }
}

struct ActuatorMessage {
    cn5: bool,
    fa10: bool,
    fa9: bool,
    cn6_1: bool,
    cn6_3: bool,
    cn6_5: bool,
    cn7: bool, // Only available on Elizabeth
    fa7: bool,
    fa8: bool,
    cn10_n_12v: bool,
    cn10_n_3v3: bool,
    minus_button: bool,
    plus_button: bool,
}

impl ActuatorMessage {
    fn to_bytes(&self) -> [u8; 5] {
        let mut bytes: [u8; 5] = [0; 5];
        bytes[0] = 0x80;

        if self.minus_button {
            bytes[3] |= 0x08;
        }
        
        if self.plus_button {
            bytes[3] |= 0x04;
        }
        
        bytes[4] = calculate_checksum(&bytes[0..4], 0x02);

        bytes
    }
}

struct Varint {
    bytes: [u8; 3]
}

impl Varint {
    fn new(bytes: [u8; 3]) -> Self {
        Varint {
            bytes
        }
    }

    fn from_u16(value: u16) -> Self {
        let byte0 = ((value >> 8) & 0xFF) as u8;
        let byte1_original = (value & 0xFF) as u8;
        let byte1 = byte1_original & 0x7F;
        let byte2 = if byte1_original & 0x80 != 0 { 0x7F } else { 0x00 };

        Self {
            bytes: [byte0, byte1, byte2],
        }
    }

    fn to_u16(&self) -> u16 {
        if self.bytes[2] == 0x7F {
            ((self.bytes[1] | 0x80) as u16) + (self.bytes[0] as u16) << 8
        } else {
            (self.bytes[1] as u16) + (self.bytes[0] as u16) << 8
        }
    }
}

fn calculate_checksum(buf: &[u8], initial: u8) -> u8 {
    if buf.len() > 32 {
        return 0;
    }

    let mut checksum: u16 = initial as u16;
    for &item in buf.iter() {
        checksum += item as u16;
    }

    (checksum & 0x7F) as u8
}