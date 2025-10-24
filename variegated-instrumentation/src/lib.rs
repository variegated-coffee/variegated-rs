#![no_std]

#[macro_export]
macro_rules! async_task_loop {
    ($name:expr, $delay:expr, $body:block) => {
        {
            let mut last_log_time = ::embassy_time::Instant::now();
            
            loop {
                // $name loop
                
                // Capture start time
                let start_time = ::embassy_time::Instant::now();
                
                $body
                
                // Calculate execution time
                let elapsed = start_time.elapsed();
                
                // Only log if at least 1 second has passed since last log
                let now = ::embassy_time::Instant::now();
                if now.duration_since(last_log_time).as_secs() >= 1 {
                    //::defmt::debug!("{} loop: {} ms", $name, elapsed.as_millis());
                    last_log_time = now;
                }
                
                // Delay at the bottom of the loop
                if let Some(delay_duration) = $delay {
                    ::embassy_time::Timer::after(delay_duration).await;
                }
            }
        }
    };
}

#[macro_export]
macro_rules! instrumented_section {
    ($name:expr, $body:block) => {
        {
            let start_time = ::embassy_time::Instant::now();

            let foo = $body;

            let elapsed = start_time.elapsed();
            // ::defmt::debug!("{} section: {} ms", $name, elapsed.as_millis());

            foo
        }
    };
}