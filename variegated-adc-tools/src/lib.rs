#![no_std]

use micromath::F32Ext;

pub struct LinearConversionParameters {
    pub k: f32,
    pub m: f32,
}

pub struct SteinhartHartParameters {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

pub struct ThermistorParameters {
    pub r25: f32,
    pub beta: f32,
}

#[derive(PartialEq, Copy, Clone)]
pub enum ResistorDividerPosition {
    Upstream,
    Downstream,
}

pub struct ResistorDividerConversionParameters {
    pub v_in: f32,
    pub r_known: f32,
    pub known_resistance_position: ResistorDividerPosition,
}

pub struct ConversionParameters {
    pub linear_conversion_parameters: Option<LinearConversionParameters>,
    pub steinhart_hart_parameters: Option<SteinhartHartParameters>,
    pub thermistor_parameters: Option<ThermistorParameters>,
    pub resistor_divider: Option<ResistorDividerConversionParameters>,
}

impl ConversionParameters {
    pub fn linear_conversion(k: f32, m: f32) -> Self {
        Self {
            linear_conversion_parameters: Some(LinearConversionParameters { k, m }),
            steinhart_hart_parameters: None,
            thermistor_parameters: None,
            resistor_divider: None,
        }
    }

    pub fn linear_conversion_with_resistor_divider(k: f32, m: f32, v_in: f32, r_known: f32, known_resistance_position: ResistorDividerPosition) -> Self {
        Self {
            linear_conversion_parameters: Some(LinearConversionParameters { k, m }),
            steinhart_hart_parameters: None,
            thermistor_parameters: None,
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
        }
    }

    pub fn steinhart_hart(a: f32, b: f32, c: f32) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: Some(SteinhartHartParameters { a, b, c }),
            thermistor_parameters: None,
            resistor_divider: None,
        }
    }

    pub fn steinhart_hart_with_resistor_divider(a: f32, b: f32, c: f32, v_in: f32, r_known: f32, known_resistance_position: ResistorDividerPosition) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: Some(SteinhartHartParameters { a, b, c }),
            thermistor_parameters: None,
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
        }
    }

    pub fn thermistor(r25: f32, beta: f32) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: None,
            thermistor_parameters: Some(ThermistorParameters { r25, beta }),
            resistor_divider: None,
        }
    }

    pub fn thermistor_with_resistor_divider(r25: f32, beta: f32, v_in: f32, r_known: f32, known_resistance_position: ResistorDividerPosition) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: None,
            thermistor_parameters: Some(ThermistorParameters { r25, beta }),
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
        }
    }

    pub fn convert(&self, value: f32) -> f32 {
        // First, handle the resistor divider calculation if present
        let resistance = if let Some(ref resistor_divider) = self.resistor_divider {
            let r = resistor_divider.r_known;
            let v_in = resistor_divider.v_in;
            let v_out = value;

            if resistor_divider.known_resistance_position == ResistorDividerPosition::Upstream {
                r * (v_in / v_out - 1.0)
            } else {
                r / (v_in / v_out - 1.0)
            }
        } else {
            value
        };

        // Then, apply the appropriate conversion
        if let Some(ref linear) = self.linear_conversion_parameters {
            // Linear conversion: y = kx + m
            linear.k * resistance + linear.m
        } else if let Some(ref sh) = self.steinhart_hart_parameters {
            // Steinhart-Hart equation: 1/T = A + B*ln(R) + C*(ln(R))^3
            let ln_r = resistance.ln();
            1.0 / (sh.a + sh.b * ln_r + sh.c * ln_r.powi(3)) - 273.15 // Convert Kelvin to Celsius
        } else if let Some(ref thermistor) = self.thermistor_parameters {
            // Beta equation: 1/T = 1/T0 + (1/B) * ln(R/R0)
            // Where T0 is 25°C (298.15K), R0 is R25
            const T0: f32 = 298.15; // 25°C in Kelvin
            let inv_t = 1.0 / T0 + (1.0 / thermistor.beta) * (resistance / thermistor.r25).ln();
            1.0 / inv_t - 273.15 // Convert Kelvin to Celsius
        } else {
            resistance  // If no conversion parameters are set, return the resistance
        }
    }
}