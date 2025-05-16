# Variegated HAL

Espresso machine primitives.

## Structure

### variegated_hal::*

* Type aliases for real-world types to achieve increased readability
  * Temperature
  * Pressure
  * Water level
  * Flow rate
  * Weight
  * Frequency
  * RPM
  * DutyCycle
  * Valve openness
  * Mixing proportions

### variegated_hal::Boiler
A boiler abstraction. Composed.

### variegated_hal::Group
A espresso machine group abstraction. Composed.

### variegated_hal::SteamWand
A steam wand abstraction. Composed.

### variegated_hal::WaterTap
A water tap abstraction. Composed.

### variegated_hal::WaterTank (should exist)

### variegated_hal::WithTask (trait)
A trait for types that have a task. A task is an async function that runs in the 
background and is responsible for handling events and updating the state of the type.

### variegated_hal::HeatingElement (trait)
Uses async_trait. A trait providing a standardized interface for types representing a heating element.

### variegated_hal::BrewMechanism (trait)
Uses async_trait. A trait providing a standardized interface for types representing a brew mechanism.

### variegated_hal::adc::ads124s08
This module contains espresso machine primitives for the ADS124S08 ADC.

### variegated_hal::machine_mechanism::single_boiler_mechanism::SingleBoilerMechanism
Single boilers typically don't have separate mechanisms for brewing, steaming and water taps. This
abstraction handles the boiler, group, steam wand and water tap as a single mechanism, and as such
solves issues of prioritizing pumping water for the steam tap (while a three-way solenoid is closed)
and brewing - ensuring that only one of these actions can happen simultaneously.