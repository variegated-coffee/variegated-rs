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

pub struct CallendarVanDusenParameters {
    pub r0: f32, // Resistance at 0°C
    pub a: f32,  // Coefficient A
    pub b: f32,  // Coefficient B
    pub c: Option<f32>, // Coefficient C (used for temperatures below 0°C)
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
    pub callendar_van_dusen_parameters: Option<CallendarVanDusenParameters>,
}

impl ConversionParameters {
    pub fn linear_conversion(k: f32, m: f32) -> Self {
        Self {
            linear_conversion_parameters: Some(LinearConversionParameters { k, m }),
            steinhart_hart_parameters: None,
            thermistor_parameters: None,
            resistor_divider: None,
            callendar_van_dusen_parameters: None,
        }
    }

    pub fn linear_conversion_with_resistor_divider(k: f32, m: f32, v_in: f32, r_known: f32, known_resistance_position: ResistorDividerPosition) -> Self {
        Self {
            linear_conversion_parameters: Some(LinearConversionParameters { k, m }),
            steinhart_hart_parameters: None,
            thermistor_parameters: None,
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
            callendar_van_dusen_parameters: None,
        }
    }

    pub fn steinhart_hart(a: f32, b: f32, c: f32) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: Some(SteinhartHartParameters { a, b, c }),
            thermistor_parameters: None,
            resistor_divider: None,
            callendar_van_dusen_parameters: None,
        }
    }

    pub fn steinhart_hart_with_resistor_divider(a: f32, b: f32, c: f32, v_in: f32, r_known: f32, known_resistance_position: ResistorDividerPosition) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: Some(SteinhartHartParameters { a, b, c }),
            thermistor_parameters: None,
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
            callendar_van_dusen_parameters: None,
        }
    }

    pub fn thermistor(r25: f32, beta: f32) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: None,
            thermistor_parameters: Some(ThermistorParameters { r25, beta }),
            resistor_divider: None,
            callendar_van_dusen_parameters: None,
        }
    }

    pub fn thermistor_with_resistor_divider(r25: f32, beta: f32, v_in: f32, r_known: f32, known_resistance_position: ResistorDividerPosition) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: None,
            thermistor_parameters: Some(ThermistorParameters { r25, beta }),
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
            callendar_van_dusen_parameters: None,
        }
    }

    pub fn callendar_van_dusen(r0: f32, a: f32, b: f32, c: Option<f32>) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: None,
            thermistor_parameters: None,
            resistor_divider: None,
            callendar_van_dusen_parameters: Some(CallendarVanDusenParameters { r0, a, b, c }),
        }
    }

    pub fn callendar_van_dusen_with_resistor_divider(
        r0: f32,
        a: f32,
        b: f32,
        c: Option<f32>,
        v_in: f32,
        r_known: f32,
        known_resistance_position: ResistorDividerPosition,
    ) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: None,
            thermistor_parameters: None,
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
            callendar_van_dusen_parameters: Some(CallendarVanDusenParameters { r0, a, b, c }),
        }
    }

    /// Creates conversion parameters for a PT100 sensor using standard IEC 60751 coefficients
    /// A = 3.9083e-3, B = -5.775e-7, C = -4.183e-12 (for T < 0°C)
    pub fn pt100() -> Self {
        Self::callendar_van_dusen(100.0, 3.9083e-3, -5.775e-7, Some(-4.183e-12))
    }

    /// Creates conversion parameters for a PT1000 sensor using standard IEC 60751 coefficients
    /// A = 3.9083e-3, B = -5.775e-7, C = -4.183e-12 (for T < 0°C)
    pub fn pt1000() -> Self {
        Self::callendar_van_dusen(1000.0, 3.9083e-3, -5.775e-7, Some(-4.183e-12))
    }

    pub fn linear_range_mapping(min_value: f32, max_value: f32, min_output: f32, max_output: f32) -> Self {
        let k = (max_output - min_output) / (max_value - min_value);
        let m = min_output - k * min_value;
        Self::linear_conversion(k, m)
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
        } else if let Some(ref cvd) = self.callendar_van_dusen_parameters {
            // Callendar-Van Dusen equation:
            // For T >= 0°C: R = R0 * (1 + A*T + B*T^2)
            // For T < 0°C: R = R0 * (1 + A*T + B*T^2 + C*(T - 100)*T^3)
            let r = resistance;
            let r0 = cvd.r0;
            let a = cvd.a;
            let b = cvd.b;

            if r >= r0 {
                // Solve quadratic equation for T >= 0°C: 0 = B*T^2 + A*T + (1 - R/R0)
                let discriminant = a * a - 4.0 * b * (1.0 - r / r0);
                if discriminant >= 0.0 {
                    (-a + discriminant.sqrt()) / (2.0 * b)
                } else {
                    f32::NAN // No real solution
                }
            } else if let Some(c) = cvd.c {
                // Solve cubic equation for T < 0°C (approximation)
                // Iterative numerical methods would be required for exact solutions.
                // Here, we use a simple approximation for small temperature ranges.
                let mut t = -200.0; // Initial guess for T < 0°C
                for _ in 0..10 {
                    let f = r0 * (1.0 + a * t + b * t * t + c * (t - 100.0) * t * t * t) - r;
                    let df = r0 * (a + 2.0 * b * t + 3.0 * c * (t - 100.0) * t * t + c * 3.0 * t * t);
                    t -= f / df;
                }
                t
            } else {
                f32::NAN // C coefficient is required for T < 0°C
            }
        } else {
            resistance  // If no conversion parameters are set, return the resistance
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_linear_conversion() {
        let params = ConversionParameters::linear_conversion(2.0, 3.0);
        let result = params.convert(5.0);
        assert_eq!(result, 13.0); // 2 * 5 + 3 = 13
    }

    #[test]
    fn test_steinhart_hart_conversion() {
        let params = ConversionParameters::steinhart_hart(0.001129148, 0.000234125, 0.0000000876741);
        let resistance = 10000.0; // Example resistance in ohms
        let result = params.convert(resistance);
        assert!((result - 25.0).abs() < 0.1); // Expect approximately 25°C
    }

    #[test]
    fn test_thermistor_conversion() {
        let params = ConversionParameters::thermistor(10000.0, 3950.0); // R25 = 10kΩ, Beta = 3950
        let resistance = 10000.0; // Example resistance in ohms
        let result = params.convert(resistance);
        assert!((result - 25.0).abs() < 0.1); // Expect approximately 25°C
    }

    #[test]
    fn test_linear_conversion_with_resistor_divider() {
        let params = ConversionParameters::linear_conversion_with_resistor_divider(
            2.0, 3.0, 5.0, 10000.0, ResistorDividerPosition::Upstream,
        );
        let v_out = 2.5; // Example voltage output
        let result = params.convert(v_out);
        assert!((result - 20003.0).abs() < 0.1); // Expected result after applying resistor divider and linear conversion
    }

    #[test]
    fn test_steinhart_hart_with_resistor_divider() {
        let params = ConversionParameters::steinhart_hart_with_resistor_divider(
            0.001129148, 0.000234125, 0.0000000876741, 5.0, 10000.0, ResistorDividerPosition::Downstream,
        );
        let v_out = 2.5; // Example voltage output
        let result = params.convert(v_out);
        assert!((result - 25.0).abs() < 0.1); // Expect approximately 25°C
    }

    #[test]
    fn test_thermistor_with_resistor_divider() {
        let params = ConversionParameters::thermistor_with_resistor_divider(
            10000.0, 3950.0, 5.0, 10000.0, ResistorDividerPosition::Upstream,
        );
        let v_out = 2.5; // Example voltage output
        let result = params.convert(v_out);
        assert!((result - 25.0).abs() < 0.1); // Expect approximately 25°C
    }

    #[test]
    fn test_callendar_van_dusen_conversion() {
        let params = ConversionParameters::callendar_van_dusen(100.0, 0.00385, -0.0000005775, None);
        let resistance = 138.5; // Example resistance for 100°C
        let result = params.convert(resistance);
        assert!((result - 100.0).abs() < 0.1); // Expect approximately 100°C
    }

    #[test]
    fn test_pt100_conversion() {
        let params = ConversionParameters::pt100();
        let resistance = 138.5; // Resistance at 100°C
        let result = params.convert(resistance);
        assert!((result - 100.0).abs() < 0.5);
        
        // Test negative temperature
        let resistance_neg = 80.31; // Approximate resistance at -50°C
        let result_neg = params.convert(resistance_neg);
        assert!((result_neg - (-50.0)).abs() < 0.5);
    }
    
    #[test]
    fn test_pt1000_conversion() {
        let params = ConversionParameters::pt1000();
        let resistance = 1385.0; // Resistance at 100°C
        let result = params.convert(resistance);
        assert!((result - 100.0).abs() < 0.5);
    }

}

