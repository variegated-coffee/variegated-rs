# Mock Data for Frontend Development

This directory contains mock JSON data files that are served by the Vite dev server during local development. This allows you to develop and test the frontend without needing to flash the ESP32 MCU or run the actual backend.

## Files

- **status.json** - Mock machine status with sensor readings, brewing state, etc.
- **configuration.json** - Mock configuration including PID parameters, control curves, schedules
- **machine-definition.json** - Mock hardware capabilities and entity definitions
- **routines.json** - Sample routines (heat-up, espresso profiles, cleaning)

## How It Works

The Vite dev server (configured in `vite.config.ts`) uses a custom middleware plugin that intercepts API requests and serves these JSON files:

- `GET /status` → serves `status.json`
- `GET /configuration` → serves `configuration.json`
- `GET /machine-definition` → serves `machine-definition.json`
- `GET /routines` → serves `routines.json`

## Usage

Simply run the dev server:

```bash
npm run dev
```

The frontend will automatically fetch from these mock endpoints. You'll see log messages in the console like:

```
[mock-data] Served /status from status.json
```

## Customizing Mock Data

You can edit any of these JSON files to test different scenarios:

- **Test brewing states**: Modify `group_statuses.group_1.is_brewing` in `status.json`
- **Test different PID values**: Edit PID parameters in `configuration.json`
- **Test configuration changes**: Modify any configuration values and reload the page
- **Add new routines**: Add entries to the `routines.json` array

The dev server will hot-reload, so changes to mock data are reflected immediately in the browser.

## Realistic Test Data

The mock data includes:
- 2 boilers (brew + steam) with realistic temperature/pressure readings
- 2 groups with different control modes (Pressure vs PressureCurve)
- One group actively brewing, one idle with previous brew data
- PID parameters with realistic values
- Kalman filter configurations
- Multiple sample routines (espresso, turbo shot, backflush cleaning)
- Schedules for auto-on/off

This provides a comprehensive test environment for frontend development.
