#![no_std]

use core::cell::RefCell;
use micromath::F32Ext;
use heapless::Vec;

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
    kalman_filter: Option<KalmanFilter>,
    median_filter: Option<MedianFilter>,
}

impl ConversionParameters {
    pub fn linear_conversion(k: f32, m: f32) -> Self {
        Self {
            linear_conversion_parameters: Some(LinearConversionParameters { k, m }),
            steinhart_hart_parameters: None,
            thermistor_parameters: None,
            resistor_divider: None,
            callendar_van_dusen_parameters: None,
            kalman_filter: None,
            median_filter: None,
        }
    }

    pub fn linear_conversion_with_resistor_divider(k: f32, m: f32, v_in: f32, r_known: f32, known_resistance_position: ResistorDividerPosition) -> Self {
        Self {
            linear_conversion_parameters: Some(LinearConversionParameters { k, m }),
            steinhart_hart_parameters: None,
            thermistor_parameters: None,
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
            callendar_van_dusen_parameters: None,
            kalman_filter: None,
            median_filter: None,
        }
    }

    pub fn steinhart_hart(a: f32, b: f32, c: f32) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: Some(SteinhartHartParameters { a, b, c }),
            thermistor_parameters: None,
            resistor_divider: None,
            callendar_van_dusen_parameters: None,
            kalman_filter: None,
            median_filter: None,
        }
    }

    pub fn steinhart_hart_with_resistor_divider(a: f32, b: f32, c: f32, v_in: f32, r_known: f32, known_resistance_position: ResistorDividerPosition) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: Some(SteinhartHartParameters { a, b, c }),
            thermistor_parameters: None,
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
            callendar_van_dusen_parameters: None,
            kalman_filter: None,
            median_filter: None,
        }
    }

    pub fn thermistor(r25: f32, beta: f32) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: None,
            thermistor_parameters: Some(ThermistorParameters { r25, beta }),
            resistor_divider: None,
            callendar_van_dusen_parameters: None,
            kalman_filter: None,
            median_filter: None,
        }
    }

    pub fn thermistor_with_resistor_divider(r25: f32, beta: f32, v_in: f32, r_known: f32, known_resistance_position: ResistorDividerPosition) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: None,
            thermistor_parameters: Some(ThermistorParameters { r25, beta }),
            resistor_divider: Some(ResistorDividerConversionParameters { v_in, r_known, known_resistance_position }),
            callendar_van_dusen_parameters: None,
            kalman_filter: None,
            median_filter: None,
        }
    }

    pub fn callendar_van_dusen(r0: f32, a: f32, b: f32, c: Option<f32>) -> Self {
        Self {
            linear_conversion_parameters: None,
            steinhart_hart_parameters: None,
            thermistor_parameters: None,
            resistor_divider: None,
            callendar_van_dusen_parameters: Some(CallendarVanDusenParameters { r0, a, b, c }),
            kalman_filter: None,
            median_filter: None,
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
            kalman_filter: None,
            median_filter: None,
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

    /// Adds Kalman filtering to existing conversion parameters
    /// 
    /// # Arguments
    /// * `process_noise` - How much the true value is expected to change (Q)
    /// * `measurement_noise` - Expected measurement noise level (R)
    /// * `initial_uncertainty` - Initial uncertainty in estimates (P0)
    pub fn with_kalman_filter(mut self, process_noise: f32, measurement_noise: f32, initial_uncertainty: f32) -> Self {
        self.kalman_filter = Some(KalmanFilter::new(process_noise, measurement_noise, initial_uncertainty));
        self
    }

    /// Adds Kalman filtering using predefined parameter sets
    pub fn with_kalman_preset(mut self, params: KalmanFilterParameters) -> Self {
        self.kalman_filter = Some(KalmanFilter::new(
            params.process_noise,
            params.measurement_noise,
            params.initial_uncertainty,
        ));
        self
    }

    /// Adds median filtering for outlier rejection
    /// 
    /// # Arguments
    /// * `window_size` - Size of the sliding window (must be odd, 3-15 recommended)
    pub fn with_median_filter(mut self, window_size: usize) -> Self {
        self.median_filter = Some(MedianFilter::new(window_size));
        self
    }

    /// Resets the Kalman filter state (useful when sensor is reconnected)
    pub fn reset_kalman_filter(&self) {
        if let Some(ref filter) = self.kalman_filter {
            filter.reset();
        }
    }

    /// Resets the median filter state (useful when sensor is reconnected)
    pub fn reset_median_filter(&self) {
        if let Some(ref filter) = self.median_filter {
            filter.reset();
        }
    }

    pub fn convert(&self, value: f32) -> f32 {
        // Apply median filtering first for outlier rejection
        let filtered_value = if let Some(ref median_filter) = self.median_filter {
            median_filter.update(value)
        } else {
            value
        };

        // Then, handle the resistor divider calculation if present
        let resistance = if let Some(ref resistor_divider) = self.resistor_divider {
            let r = resistor_divider.r_known;
            let v_in = resistor_divider.v_in;
            let v_out = filtered_value;

            if resistor_divider.known_resistance_position == ResistorDividerPosition::Upstream {
                r * (v_in / v_out - 1.0)
            } else {
                r / (v_in / v_out - 1.0)
            }
        } else {
            filtered_value
        };

        // Then, apply the appropriate conversion
        let converted_value = if let Some(ref linear) = self.linear_conversion_parameters {
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
        };

        // Apply Kalman filtering if configured
        if let Some(ref filter) = self.kalman_filter {
            filter.update(converted_value)
        } else {
            converted_value
        }
    }
}

/// Median filter for outlier rejection
pub struct MedianFilter {
    buffer: RefCell<Vec<f32, 15>>, // Maximum window size of 15
    window_size: usize,
    initialized: RefCell<bool>,
}

impl MedianFilter {
    /// Creates a new median filter with specified window size
    /// 
    /// # Arguments
    /// * `window_size` - Size of the sliding window (must be odd and <= 15)
    /// 
    /// # Panics
    /// Panics if window_size is even or greater than 15
    pub fn new(window_size: usize) -> Self {
        assert!(window_size > 0 && window_size <= 15, "Window size must be between 1 and 15");
        assert!(window_size % 2 == 1, "Window size must be odd");
        
        Self {
            buffer: RefCell::new(Vec::new()),
            window_size,
            initialized: RefCell::new(false),
        }
    }

    /// Updates the filter with a new measurement and returns the median
    pub fn update(&self, measurement: f32) -> f32 {
        let mut buffer = self.buffer.borrow_mut();
        let mut initialized = self.initialized.borrow_mut();
        
        // Add new measurement
        if buffer.len() < self.window_size {
            buffer.push(measurement).ok(); // Ignore error if full (shouldn't happen)
        } else {
            // Shift buffer and add new value
            for i in 0..self.window_size - 1 {
                buffer[i] = buffer[i + 1];
            }
            buffer[self.window_size - 1] = measurement;
        }
        
        if !*initialized && buffer.len() < self.window_size {
            // Not enough samples yet, return the measurement as-is
            measurement
        } else {
            *initialized = true;
            // Calculate median
            self.calculate_median(&buffer)
        }
    }

    fn calculate_median(&self, buffer: &Vec<f32, 15>) -> f32 {
        let mut sorted = Vec::<f32, 15>::new();
        for &value in buffer {
            sorted.push(value).ok(); // Safe because we control buffer size
        }
        
        // Simple bubble sort (efficient for small arrays)
        for i in 0..sorted.len() {
            for j in 0..sorted.len() - 1 - i {
                if sorted[j] > sorted[j + 1] {
                    sorted.swap(j, j + 1);
                }
            }
        }
        
        let len = sorted.len();
        if len % 2 == 1 {
            sorted[len / 2]
        } else {
            (sorted[len / 2 - 1] + sorted[len / 2]) / 2.0
        }
    }

    /// Resets the filter state
    pub fn reset(&self) {
        self.buffer.borrow_mut().clear();
        *self.initialized.borrow_mut() = false;
    }

    /// Gets the current buffer size
    pub fn current_size(&self) -> usize {
        self.buffer.borrow().len()
    }
}

/// Simple 1D Kalman filter for sensor noise reduction
pub struct KalmanFilter {
    state: RefCell<KalmanState>,
    process_noise: f32,      // Q - process noise covariance
    measurement_noise: f32,  // R - measurement noise covariance
}

#[derive(Clone, Copy)]
struct KalmanState {
    estimate: f32,           // x - current estimate
    error_covariance: f32,   // P - error covariance
    initialized: bool,
}

impl KalmanFilter {
    /// Creates a new Kalman filter with specified noise parameters
    /// 
    /// # Arguments
    /// * `process_noise` - Q: How much we expect the true value to change between measurements
    /// * `measurement_noise` - R: How much noise we expect in measurements
    /// * `initial_uncertainty` - P0: Initial uncertainty in our estimate
    pub fn new(process_noise: f32, measurement_noise: f32, initial_uncertainty: f32) -> Self {
        Self {
            state: RefCell::new(KalmanState {
                estimate: 0.0,
                error_covariance: initial_uncertainty,
                initialized: false,
            }),
            process_noise,
            measurement_noise,
        }
    }

    /// Updates the filter with a new measurement and returns the filtered estimate
    pub fn update(&self, measurement: f32) -> f32 {
        let mut state = self.state.borrow_mut();
        
        if !state.initialized {
            // Initialize with first measurement
            state.estimate = measurement;
            state.initialized = true;
            return measurement;
        }

        // Prediction step
        let predicted_error_covariance = state.error_covariance + self.process_noise;

        // Update step
        let kalman_gain = predicted_error_covariance / (predicted_error_covariance + self.measurement_noise);
        let innovation = measurement - state.estimate;
        
        // Update estimate and error covariance
        state.estimate = state.estimate + kalman_gain * innovation;
        state.error_covariance = (1.0 - kalman_gain) * predicted_error_covariance;

        state.estimate
    }

    /// Resets the filter state (useful for sensor reconnection)
    pub fn reset(&self) {
        let mut state = self.state.borrow_mut();
        state.initialized = false;
        // Note: error_covariance will be reset to initial_uncertainty on next update
    }

    /// Gets the current estimate without updating (read-only)
    pub fn current_estimate(&self) -> Option<f32> {
        let state = self.state.borrow();
        if state.initialized {
            Some(state.estimate)
        } else {
            None
        }
    }
}

/// Parameters for configuring Kalman filtering in ConversionParameters
#[derive(Clone, Copy)]
pub struct KalmanFilterParameters {
    pub process_noise: f32,      // Q - process noise covariance
    pub measurement_noise: f32,  // R - measurement noise covariance
    pub initial_uncertainty: f32, // P0 - initial uncertainty
}

impl KalmanFilterParameters {
    /// Conservative filtering - slow to adapt, very stable
    pub fn conservative() -> Self {
        Self {
            process_noise: 0.001,
            measurement_noise: 0.1,
            initial_uncertainty: 1.0,
        }
    }

    /// Balanced filtering - good compromise between responsiveness and stability
    pub fn balanced() -> Self {
        Self {
            process_noise: 0.01,
            measurement_noise: 0.05,
            initial_uncertainty: 0.5,
        }
    }

    /// Responsive filtering - quick to adapt, less stable
    pub fn responsive() -> Self {
        Self {
            process_noise: 0.1,
            measurement_noise: 0.01,
            initial_uncertainty: 0.1,
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

    #[test]
    fn test_kalman_filter_basic() {
        let filter = KalmanFilter::new(0.01, 0.1, 1.0);
        
        // First measurement should be returned as-is
        let result1 = filter.update(10.0);
        assert_eq!(result1, 10.0);
        
        // Second measurement should be filtered
        let result2 = filter.update(12.0);
        assert!(result2 > 10.0 && result2 < 12.0); // Should be between the two values
        
        // Third measurement should continue filtering
        let result3 = filter.update(10.5);
        assert!(result3 > 10.0 && result3 < 12.0);
    }

    #[test]
    fn test_kalman_filter_convergence() {
        let filter = KalmanFilter::new(0.001, 0.1, 1.0); // Low process noise, high measurement noise
        
        // Feed constant value with noise
        let true_value = 25.0;
        let mut last_estimate = 0.0;
        
        for i in 0..20 {
            let noisy_measurement = true_value + (i as f32 % 5.0 - 2.0) * 0.1; // Small noise
            last_estimate = filter.update(noisy_measurement);
        }
        
        // Should converge close to true value
        assert!((last_estimate - true_value).abs() < 0.1);
    }

    #[test]
    fn test_kalman_filter_reset() {
        let filter = KalmanFilter::new(0.01, 0.1, 1.0);
        
        filter.update(10.0);
        filter.update(15.0);
        assert!(filter.current_estimate().is_some());
        
        filter.reset();
        assert!(filter.current_estimate().is_none());
        
        // First measurement after reset should be returned as-is
        let result = filter.update(20.0);
        assert_eq!(result, 20.0);
    }

    #[test]
    fn test_conversion_with_kalman_filter() {
        let params = ConversionParameters::linear_conversion(2.0, 3.0)
            .with_kalman_filter(0.01, 0.1, 1.0);
        
        // First conversion
        let result1 = params.convert(5.0); // Should be 2*5+3 = 13.0
        assert_eq!(result1, 13.0);
        
        // Second conversion should be filtered
        let result2 = params.convert(6.0); // Would be 2*6+3 = 15.0 without filtering
        assert!(result2 > 13.0 && result2 < 15.0);
    }

    #[test]
    fn test_kalman_preset_parameters() {
        let conservative = KalmanFilterParameters::conservative();
        assert_eq!(conservative.process_noise, 0.001);
        assert_eq!(conservative.measurement_noise, 0.1);
        
        let balanced = KalmanFilterParameters::balanced();
        assert_eq!(balanced.process_noise, 0.01);
        assert_eq!(balanced.measurement_noise, 0.05);
        
        let responsive = KalmanFilterParameters::responsive();
        assert_eq!(responsive.process_noise, 0.1);
        assert_eq!(responsive.measurement_noise, 0.01);
    }

    #[test]
    fn test_median_filter_basic() {
        let filter = MedianFilter::new(3);
        
        // First measurement should be returned as-is
        let result1 = filter.update(10.0);
        assert_eq!(result1, 10.0);
        
        // Second measurement should be returned as-is (not enough samples)
        let result2 = filter.update(5.0);
        assert_eq!(result2, 5.0);
        
        // Third measurement should return median
        let result3 = filter.update(8.0);
        assert_eq!(result3, 8.0); // median of [10.0, 5.0, 8.0]
        
        // Fourth measurement (sliding window)
        let result4 = filter.update(12.0);
        assert_eq!(result4, 8.0); // median of [5.0, 8.0, 12.0]
    }

    #[test]
    fn test_median_filter_outlier_rejection() {
        let filter = MedianFilter::new(5);
        
        // Fill buffer with normal values
        filter.update(10.0);
        filter.update(11.0);
        filter.update(10.5);
        filter.update(9.5);
        let result = filter.update(10.2);
        
        // Should be close to the median of normal values
        assert!((result - 10.2).abs() < 0.1);
        
        // Add an outlier
        let result_with_outlier = filter.update(50.0); // Big outlier
        
        // Should reject the outlier and stay close to normal range
        assert!(result_with_outlier < 15.0);
        assert!(result_with_outlier > 9.0);
    }

    #[test]
    fn test_median_filter_window_size_5() {
        let filter = MedianFilter::new(5);
        let values = [1.0, 3.0, 2.0, 4.0, 5.0];
        let mut last_result = 0.0;
        
        for value in values {
            last_result = filter.update(value);
        }
        
        // Should be median of [1.0, 3.0, 2.0, 4.0, 5.0] = 3.0
        assert_eq!(last_result, 3.0);
    }

    #[test]
    fn test_median_filter_reset() {
        let filter = MedianFilter::new(3);
        
        filter.update(10.0);
        filter.update(20.0);
        filter.update(15.0);
        
        assert_eq!(filter.current_size(), 3);
        
        filter.reset();
        assert_eq!(filter.current_size(), 0);
        
        // First measurement after reset should be returned as-is
        let result = filter.update(25.0);
        assert_eq!(result, 25.0);
    }

    #[test]
    fn test_conversion_with_median_filter() {
        let params = ConversionParameters::linear_conversion(2.0, 3.0)
            .with_median_filter(3);
        
        // First conversion
        let result1 = params.convert(5.0); // Should be 2*5+3 = 13.0
        assert_eq!(result1, 13.0);
        
        // Second conversion
        let result2 = params.convert(6.0); // Should be 2*6+3 = 15.0 (not enough for median yet)
        assert_eq!(result2, 15.0);
        
        // Third conversion should use median filtering
        let result3 = params.convert(7.0); // median of [5.0, 6.0, 7.0] = 6.0, then 2*6+3 = 15.0
        assert_eq!(result3, 15.0);
    }

    #[test]
    fn test_conversion_with_median_and_kalman_filter() {
        let params = ConversionParameters::linear_conversion(1.0, 0.0)
            .with_median_filter(3)
            .with_kalman_filter(0.01, 0.1, 1.0);
        
        params.convert(10.0); // First value
        params.convert(11.0); // Second value  
        let result = params.convert(50.0); // Outlier - should be filtered by median first
        
        // Result should be much less than 50 due to median filtering
        assert!(result < 20.0);
        assert!(result > 5.0);
    }

    #[test]
    fn test_conversion_with_kalman_preset() {
        let params = ConversionParameters::pt100()
            .with_kalman_preset(KalmanFilterParameters::balanced());
        
        // Test that it works with PT100 conversion
        let resistance = 138.5; // Resistance at 100°C
        let result = params.convert(resistance);
        assert!((result - 100.0).abs() < 0.5);
        
        // Test filtering works
        let result2 = params.convert(140.0);
        assert!(result2 > 100.0); // Should have moved toward new value
    }

    #[test]
    fn test_kalman_filter_reset_method() {
        let params = ConversionParameters::linear_conversion(1.0, 0.0)
            .with_kalman_filter(0.01, 0.1, 1.0);
        
        params.convert(10.0);
        params.convert(12.0);
        
        // Reset filter
        params.reset_kalman_filter();
        
        // Next conversion should behave as first measurement
        let result = params.convert(15.0);
        assert_eq!(result, 15.0);
    }

}

