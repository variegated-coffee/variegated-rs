# Variegated Instrumentation

A lightweight performance instrumentation library for embedded systems.

## Features

- **Zero-cost when disabled**: All instrumentation compiles to nothing without the `instrumentation` feature
- **Lock-free counters and indicators**: Uses atomic operations for minimal overhead
- **Type-safe IDs**: Counter and indicator IDs defined via enums with `Into<u8>` implementation
- **No allocation**: All storage is static and compile-time sized
- **Single writer per metric**: Each counter/indicator is written by one task, readable from anywhere
- **Separate ID spaces**: Counters and indicators have independent ID ranges (0-255 each)

## Performance Counters

Counters track **events** by incrementing. Ideal for counting occurrences (loop iterations, errors, sensor reads, etc.).

### Counter Usage

```ignore
use variegated_instrumentation::{PerformanceCounters, define_counters};

define_counters! {
    enum CounterId {
        LoopIterations = 0,
        SensorReads = 1,
        Errors = 2,
    }
}

static COUNTERS: PerformanceCounters<3> = PerformanceCounters::new();

#[embassy_executor::task]
async fn my_task() {
    let handle = COUNTERS.handle(CounterId::LoopIterations);
    loop {
        handle.increment();
        // ... work ...
    }
}

// Reading counters and calculating rates
#[embassy_executor::task]
async fn monitor_task() {
    let mut last_count = 0;
    let mut last_time = Instant::now();

    loop {
        Timer::after_secs(5).await;

        let count = COUNTERS.read(CounterId::LoopIterations);
        let now = Instant::now();

        let delta_count = count - last_count;
        let delta_time = now - last_time;
        let rate = delta_count as f32 / delta_time.as_secs_f32();

        log_info!("Rate: {:.1}/s", rate);

        last_count = count;
        last_time = now;
    }
}
```

## Performance Indicators

Indicators track **current state** by setting values. Ideal for measurements (temperature, pressure, queue depth, memory usage, etc.).

### Indicator Usage

```ignore
use variegated_instrumentation::{PerformanceIndicators, define_indicators};

define_indicators! {
    enum IndicatorId {
        BoilerTemp = 0,
        GroupPressure = 1,
        FlowRate = 2,
    }
}

static INDICATORS: PerformanceIndicators<3> = PerformanceIndicators::new();

#[embassy_executor::task]
async fn sensor_task() {
    let temp_handle = INDICATORS.handle(IndicatorId::BoilerTemp);
    let pressure_handle = INDICATORS.handle(IndicatorId::GroupPressure);

    loop {
        let temp = read_temperature().await;
        let pressure = read_pressure().await;

        temp_handle.set(temp as u64);
        pressure_handle.set(pressure as u64);

        Timer::after_millis(100).await;
    }
}

// Reading indicators
#[embassy_executor::task]
async fn monitor_task() {
    loop {
        let temp = INDICATORS.read(IndicatorId::BoilerTemp);
        let pressure = INDICATORS.read(IndicatorId::GroupPressure);

        log_info!("Temp: {}°C, Pressure: {} bar", temp, pressure);

        Timer::after_secs(1).await;
    }
}
```

## Counters vs Indicators

| Feature | Counters | Indicators |
|---------|----------|------------|
| **Operation** | `increment()`, `add(n)` | `set(value)` |
| **Purpose** | Track events/occurrences | Track current state |
| **Typical use** | Loop iterations, error counts | Temperature, pressure, queue depth |
| **Value semantics** | Monotonically increasing | Set to specific value |
| **ID space** | 0-255 (separate) | 0-255 (separate) |

## Cargo Features

- `instrumentation`: Enable performance counters and indicators (disabled by default for zero cost)
- `defmt`: Enable defmt formatting support
- `serde`: Enable serialization support

## Memory Usage

Both counters and indicators use 8 bytes (AtomicU64) per metric:
- **50 metrics**: 400 bytes
- **100 metrics**: 800 bytes
