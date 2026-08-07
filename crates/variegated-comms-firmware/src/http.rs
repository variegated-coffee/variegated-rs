//! HTTP server functionality

use alloc::collections::BTreeMap;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt::{Debug, Display};
use core::net::SocketAddr;

// Embedded frontend files
static INDEX_HTML: &[u8] = include_bytes!("../../../frontend/dist/index.html");
static APP_JS_GZ: &[u8] = include_bytes!("../../../frontend/dist/assets/index.js.gz");

// No `log_warn`: this file's only `warn!` site is the one at line ~1293 that had to
// stay on `defmt`, because `defmt::Debug2Format` implements `Debug` but not
// `Display` and that site formats it with `{}`.
use variegated_log::{log_error, log_info};
use edge_http::io::server::{Connection as ServerConnection, DefaultServer, Handler};
use edge_http::io::Error;
use edge_http::Method;
use edge_nal::TcpBind;
use edge_nal_embassy::Tcp;
use embedded_io_async::{Read, Write};
use embassy_futures::select::{select, Either};

use variegated_controller_types::{
    BoilerControlTargetValuesUpdate, Configuration, GroupBrewControlTargetValuesUpdate,
    MachineCommand, MachineMode, PidParameterTarget, Routine, RoutineIndex, RoutineList,
    ScheduleItem, Status,
};

use crate::api_types::{
    RoutineStorage, SetBoilerControlRequest, SetFillPumpConfigurationRequest,
    SetGroupControlRequest, SetGroupPumpConfigurationRequest, SetPidParametersRequest,
    SetSteamValveOpennessRequest, SetWaterTapPumpConfigurationRequest,
};
use crate::channels::{
    ApplicationConfigurationSubscriber, ApplicationStatusSubscriber, MachineCommandSender,
    CONFIG_CACHE, MACHINE_DEFINITION, ROUTINE_CACHE, STATUS_CACHE,
};

/// HTTP request handler
pub struct HttpHandler {
    command_sender: &'static MachineCommandSender,
}

impl HttpHandler {
    pub fn new(command_sender: &'static MachineCommandSender) -> Self {
        Self { command_sender }
    }

    // Helper to send a simple response
    async fn send_response<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        status: u16,
        reason: &str,
        content_type: &str,
        body: &[u8],
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        conn.initiate_response(status, Some(reason), &[("Content-Type", content_type)])
            .await?;
        conn.write_all(body).await?;
        Ok(())
    }

    // Helper for binary (postcard) responses
    async fn send_binary<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        data: &[u8],
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_response(conn, 200, "OK", "application/octet-stream", data).await
    }

    // Helper for text responses
    async fn send_text<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        status: u16,
        reason: &str,
        body: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_response(conn, status, reason, "text/plain", body.as_bytes()).await
    }

    // Helper for 404 response
    async fn send_not_found<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_text(conn, 404, "Not Found", "Not found").await
    }

    // Helper for 503 response (service unavailable)
    async fn send_unavailable<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        message: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_text(conn, 503, "Service Unavailable", message).await
    }

    // Helper for 400 response (bad request)
    async fn send_bad_request<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        message: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_text(conn, 400, "Bad Request", message).await
    }

    // Helper for 500 response (internal server error)
    async fn send_internal_error<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        message: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_text(conn, 500, "Internal Server Error", message).await
    }

    // Read request body into a Vec
    async fn read_body<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        max_size: usize,
    ) -> Result<Vec<u8>, Error<T::Error>>
    where
        T: Read + Write,
    {
        let mut buf = vec![0u8; max_size];
        let mut total_read = 0;

        loop {
            match conn.read(&mut buf[total_read..]).await {
                Ok(0) => break,
                Ok(n) => {
                    total_read += n;
                    if total_read >= max_size {
                        break;
                    }
                }
                Err(e) => return Err(e),
            }
        }

        buf.truncate(total_read);
        Ok(buf)
    }

    // GET /status
    async fn handle_get_status<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET /status");

        let status = STATUS_CACHE.lock().await;
        if let Some(ref s) = *status {
            match postcard::to_allocvec(s) {
                Ok(binary) => {
                    drop(status);
                    Self::send_binary(conn, &binary).await
                }
                Err(e) => {
                    log_error!("Failed to serialize status: {:?}", defmt::Debug2Format(&e));
                    Self::send_internal_error(conn, "Failed to serialize status").await
                }
            }
        } else {
            Self::send_unavailable(conn, "Status not yet available").await
        }
    }

    // GET /configuration
    async fn handle_get_configuration<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET /configuration");

        let config = CONFIG_CACHE.lock().await;
        if let Some(ref c) = *config {
            match postcard::to_allocvec(c) {
                Ok(binary) => {
                    drop(config);
                    Self::send_binary(conn, &binary).await
                }
                Err(e) => {
                    log_error!("Failed to serialize configuration: {:?}", defmt::Debug2Format(&e));
                    Self::send_internal_error(conn, "Failed to serialize configuration").await
                }
            }
        } else {
            Self::send_unavailable(conn, "Configuration not yet available").await
        }
    }

    // GET /machine-definition
    async fn handle_get_machine_definition<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET /machine-definition");

        let machine_def = MACHINE_DEFINITION.lock().await;
        if let Some(ref md) = *machine_def {
            match postcard::to_allocvec(md) {
                Ok(binary) => {
                    drop(machine_def);
                    Self::send_binary(conn, &binary).await
                }
                Err(e) => {
                    log_error!("Failed to serialize machine definition: {:?}", defmt::Debug2Format(&e));
                    Self::send_internal_error(conn, "Failed to serialize machine definition").await
                }
            }
        } else {
            Self::send_unavailable(conn, "Machine definition not yet available").await
        }
    }

    // GET /routines
    async fn handle_get_routines<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET /routines");

        let routine_list = ROUTINE_CACHE.lock().await;
        if let Some(ref routines) = *routine_list {
            // Categorize routines by type
            let mut internal_map = BTreeMap::new();
            let mut function_map = BTreeMap::new();
            let mut custom_map = BTreeMap::new();

            for (routine_index, routine) in routines.routines.iter() {
                match routine_index {
                    RoutineIndex::Internal(index) => {
                        internal_map.insert(*index as u32, routine.clone());
                    }
                    RoutineIndex::Function(index) => {
                        function_map.insert(*index as u32, routine.clone());
                    }
                    RoutineIndex::Custom(index) => {
                        custom_map.insert(*index as u32, routine.clone());
                    }
                }
            }

            let storage = RoutineStorage {
                internal: internal_map,
                function: function_map,
                custom: custom_map,
            };

            match postcard::to_allocvec(&storage) {
                Ok(binary) => {
                    drop(routine_list);
                    Self::send_binary(conn, &binary).await
                }
                Err(e) => {
                    log_error!("Failed to serialize routines: {:?}", defmt::Debug2Format(&e));
                    Self::send_internal_error(conn, "Failed to serialize routines").await
                }
            }
        } else {
            Self::send_unavailable(conn, "Routines not yet available").await
        }
    }

    // POST /schedules - Add new schedule
    async fn handle_post_schedule<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /schedules");

        let body = Self::read_body(conn, 8192).await?;

        let schedule_item: ScheduleItem = match postcard::from_bytes(&body) {
            Ok(item) => item,
            Err(e) => {
                log_error!("Failed to deserialize ScheduleItem: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::AddScheduleItem(schedule_item);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Schedule add command sent");
                Self::send_text(conn, 201, "Created", "Schedule added").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // PUT /schedules/{index} - Update schedule
    async fn handle_put_schedule<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("PUT /schedules/{}", index);

        let body = Self::read_body(conn, 8192).await?;

        let schedule_item: ScheduleItem = match postcard::from_bytes(&body) {
            Ok(item) => item,
            Err(e) => {
                log_error!("Failed to deserialize ScheduleItem: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::UpdateScheduleItem(index, schedule_item);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Schedule update command sent for index {}", index);
                Self::send_text(conn, 200, "OK", "Schedule updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // DELETE /schedules/{index} - Remove schedule
    async fn handle_delete_schedule<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("DELETE /schedules/{}", index);

        let cmd = MachineCommand::RemoveScheduleItem(index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Schedule delete command sent for index {}", index);
                conn.initiate_response(204, Some("No Content"), &[]).await?;
                Ok(())
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /routines/{type} or POST /routines/{type}/{index}
    async fn handle_post_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        path: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /routines/{}", path);

        let parts: Vec<&str> = path.split('/').filter(|s| !s.is_empty()).collect();

        if parts.is_empty() || parts.len() > 2 {
            return Self::send_bad_request(conn, "Invalid path format").await;
        }

        let routine_type = parts[0];

        let body = Self::read_body(conn, 16384).await?;

        let routine: Routine = match postcard::from_bytes(&body) {
            Ok(item) => item,
            Err(e) => {
                log_error!("Failed to deserialize Routine: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = match routine_type {
            "custom" => {
                if parts.len() > 1 {
                    return Self::send_bad_request(
                        conn,
                        "Custom routines auto-assign index. Use /routines/custom without index.",
                    )
                    .await;
                }
                MachineCommand::AddRoutine(routine)
            }
            "function" => {
                if parts.len() != 2 {
                    return Self::send_bad_request(
                        conn,
                        "Function routines require index. Use /routines/function/{index}",
                    )
                    .await;
                }
                let index: u32 = match parts[1].parse() {
                    Ok(idx) => idx,
                    Err(_) => {
                        return Self::send_bad_request(conn, "Invalid index").await;
                    }
                };
                MachineCommand::UpdateRoutine(RoutineIndex::Function(index), routine)
            }
            "internal" => {
                return Self::send_bad_request(conn, "Internal routines cannot be created via API")
                    .await;
            }
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid routine type: must be 'custom' or 'function'",
                )
                .await;
            }
        };

        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Routine add command sent");
                Self::send_text(conn, 201, "Created", "Routine added").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // PUT /routines/{type}/{index}
    async fn handle_put_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        routine_type: &str,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("PUT /routines/{}/{}", routine_type, index);

        let routine_index = match routine_type {
            "internal" => RoutineIndex::Internal(index),
            "function" => RoutineIndex::Function(index),
            "custom" => RoutineIndex::Custom(index),
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid routine type: must be 'internal', 'function', or 'custom'",
                )
                .await;
            }
        };

        let body = Self::read_body(conn, 16384).await?;

        let routine: Routine = match postcard::from_bytes(&body) {
            Ok(item) => item,
            Err(e) => {
                log_error!("Failed to deserialize Routine: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::UpdateRoutine(routine_index, routine);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Routine update command sent");
                Self::send_text(conn, 200, "OK", "Routine updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // DELETE /routines/{type}/{index}
    async fn handle_delete_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        routine_type: &str,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("DELETE /routines/{}/{}", routine_type, index);

        let routine_index = match routine_type {
            "internal" => RoutineIndex::Internal(index),
            "function" => RoutineIndex::Function(index),
            "custom" => RoutineIndex::Custom(index),
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid routine type: must be 'internal', 'function', or 'custom'",
                )
                .await;
            }
        };

        let cmd = MachineCommand::RemoveRoutine(routine_index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Routine delete command sent");
                conn.initiate_response(204, Some("No Content"), &[]).await?;
                Ok(())
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/run-routine/{type}/{index}
    async fn handle_run_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        routine_type: &str,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/run-routine/{}/{}", routine_type, index);

        let routine_index = match routine_type {
            "internal" => RoutineIndex::Internal(index),
            "function" => RoutineIndex::Function(index),
            "custom" => RoutineIndex::Custom(index),
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid routine type: must be 'internal', 'function', or 'custom'",
                )
                .await;
            }
        };

        let cmd = MachineCommand::RunRoutine(routine_index, None);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Run routine command sent");
                Self::send_text(conn, 200, "OK", "Routine execution started").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/cancel-routine
    async fn handle_cancel_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/cancel-routine");

        let cmd = MachineCommand::CancelRoutine;
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Cancel routine command sent");
                Self::send_text(conn, 200, "OK", "Routine cancelled").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/tare-group-scale/{index}
    async fn handle_tare_group_scale<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u8,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/tare-group-scale/{}", index);

        let cmd = MachineCommand::TareGroupScale(index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Tare group scale command sent");
                Self::send_text(conn, 200, "OK", "Group scale tared").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/zero-calibrate-group-scale/{index}
    async fn handle_zero_calibrate_group_scale<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u8,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/zero-calibrate-group-scale/{}", index);

        let cmd = MachineCommand::ZeroCalibrateGroupScale(index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Zero calibrate group scale command sent");
                Self::send_text(conn, 200, "OK", "Group scale zero calibrated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/calibrate-group-scale-100g/{index}
    async fn handle_calibrate_group_scale_100g<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u8,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/calibrate-group-scale-100g/{}", index);

        let cmd = MachineCommand::CalibrateGroupScale100g(index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Calibrate group scale 100g command sent");
                Self::send_text(conn, 200, "OK", "Group scale calibrated with 100g").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-mode/{mode}
    async fn handle_set_mode<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        mode_str: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-mode/{}", mode_str);

        let mode = match mode_str.to_lowercase().as_str() {
            "on" => MachineMode::On,
            "off" => MachineMode::Off,
            "powersavestandby" | "power-save-standby" => MachineMode::PowerSaveStandby,
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid mode: must be 'on', 'off', or 'powersavestandby'",
                )
                .await;
            }
        };

        let cmd = MachineCommand::SetMachineMode(mode);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Set machine mode command sent");
                Self::send_text(conn, 200, "OK", "Machine mode updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-boiler-control
    async fn handle_set_boiler_control<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-boiler-control");

        let body = Self::read_body(conn, 512).await?;

        let req: SetBoilerControlRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetBoilerControlRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = if req.target_temperature.is_some() || req.target_pressure.is_some() {
            MachineCommand::SetBoilerControlTarget(
                req.boiler_index,
                req.mode,
                Some(BoilerControlTargetValuesUpdate {
                    temperature: req.target_temperature,
                    pressure: req.target_pressure,
                }),
            )
        } else {
            MachineCommand::SetBoilerControlTarget(req.boiler_index, req.mode, None)
        };

        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Boiler control command sent");
                Self::send_text(conn, 200, "OK", "Boiler control updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-group-control
    async fn handle_set_group_control<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-group-control");

        let body = Self::read_body(conn, 512).await?;

        let req: SetGroupControlRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetGroupControlRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let has_values = req.flow_rate.is_some()
            || req.pressure.is_some()
            || req.output_flow_rate.is_some()
            || req.duty_cycle.is_some()
            || req.flow_rate_curve.is_some()
            || req.pressure_curve.is_some()
            || req.output_flow_rate_curve.is_some()
            || req.duty_cycle_curve.is_some();

        let cmd = if has_values {
            MachineCommand::SetGroupBrewControlTarget(
                req.group_index,
                req.mode,
                Some(GroupBrewControlTargetValuesUpdate {
                    flow_rate: req.flow_rate,
                    flow_rate_curve: req.flow_rate_curve,
                    pressure: req.pressure,
                    pressure_curve: req.pressure_curve,
                    output_flow_rate: req.output_flow_rate,
                    output_flow_rate_curve: req.output_flow_rate_curve,
                    duty_cycle: req.duty_cycle,
                    duty_cycle_curve: req.duty_cycle_curve,
                }),
            )
        } else {
            MachineCommand::SetGroupBrewControlTarget(req.group_index, req.mode, None)
        };

        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Group control command sent");
                Self::send_text(conn, 200, "OK", "Group control updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-pid-parameters
    async fn handle_set_pid_parameters<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-pid-parameters");

        let body = Self::read_body(conn, 1024).await?;

        let req: SetPidParametersRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetPidParametersRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let target = match req.target_type.as_str() {
            "BoilerTemperature" => PidParameterTarget::BoilerTemperature(req.index as u8),
            "BoilerPressure" => PidParameterTarget::BoilerPressure(req.index as u8),
            "GroupFlowRate" => PidParameterTarget::GroupFlowRate(req.index as u8),
            "GroupOutputFlowRate" => PidParameterTarget::GroupOutputFlowRate(req.index as u8),
            "GroupPressure" => PidParameterTarget::GroupPressure(req.index as u8),
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid target_type",
                )
                .await;
            }
        };

        let cmd = MachineCommand::SetPidParameters(target, req.pid_parameters);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetPidParameters command sent");
                Self::send_text(conn, 200, "OK", "PID parameters updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-group-pump-configuration
    async fn handle_set_group_pump_configuration<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-group-pump-configuration");

        let body = Self::read_body(conn, 512).await?;

        let req: SetGroupPumpConfigurationRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetGroupPumpConfigurationRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::SetGroupPumpConfiguration(req.group_index, req.pump_configuration);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetGroupPumpConfiguration command sent");
                Self::send_text(conn, 200, "OK", "Group pump configuration updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-water-tap-pump-configuration
    async fn handle_set_water_tap_pump_configuration<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-water-tap-pump-configuration");

        let body = Self::read_body(conn, 512).await?;

        let req: SetWaterTapPumpConfigurationRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetWaterTapPumpConfigurationRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::SetWaterTapPumpConfiguration(
            req.water_tap_index,
            req.pump_configuration,
        );
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetWaterTapPumpConfiguration command sent");
                Self::send_text(conn, 200, "OK", "Water tap pump configuration updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-fill-pump-configuration
    async fn handle_set_fill_pump_configuration<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-fill-pump-configuration");

        let body = Self::read_body(conn, 512).await?;

        let req: SetFillPumpConfigurationRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetFillPumpConfigurationRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd =
            MachineCommand::SetFillPumpConfiguration(req.boiler_index, req.pump_configuration);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetFillPumpConfiguration command sent");
                Self::send_text(conn, 200, "OK", "Fill pump configuration updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-steam-valve-openness
    async fn handle_set_steam_valve_openness<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-steam-valve-openness");

        let body = Self::read_body(conn, 512).await?;

        let req: SetSteamValveOpennessRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetSteamValveOpennessRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::SetSteamValveOpenness(req.steam_wand_index, req.openness);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetSteamValveOpenness command sent");
                Self::send_text(conn, 200, "OK", "Steam valve openness updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/optimize-routine-storage
    async fn handle_optimize_routine_storage<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/optimize-routine-storage");

        let cmd = MachineCommand::OptimizeRoutineStorage;
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Optimize routine storage command sent");
                Self::send_text(conn, 200, "OK", "Routine storage optimization started").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/optimize-schedule-storage
    async fn handle_optimize_schedule_storage<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/optimize-schedule-storage");

        let cmd = MachineCommand::OptimizeScheduleStorage;
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Optimize schedule storage command sent");
                Self::send_text(conn, 200, "OK", "Schedule storage optimization started").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/optimize-configuration-storage
    async fn handle_optimize_configuration_storage<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/optimize-configuration-storage");

        let cmd = MachineCommand::OptimizeConfigurationStorage;
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Optimize configuration storage command sent");
                Self::send_text(conn, 200, "OK", "Configuration storage optimization started")
                    .await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // Parse path parameters from a path like /schedules/123
    fn parse_path_index(path: &str, prefix: &str) -> Option<u32> {
        let index_str = path.strip_prefix(prefix)?;
        index_str.parse().ok()
    }

    // Parse u8 index
    fn parse_path_u8_index(path: &str, prefix: &str) -> Option<u8> {
        let index_str = path.strip_prefix(prefix)?;
        index_str.parse().ok()
    }

    // Serve index.html
    async fn handle_index_html<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("Serving index.html");
        Self::send_response(conn, 200, "OK", "text/html; charset=utf-8", INDEX_HTML).await
    }

    // Serve JS file with Gzip encoding
    async fn handle_js_gz<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("Serving app.js.gz");
        conn.initiate_response(
            200,
            Some("OK"),
            &[
                ("Content-Type", "application/javascript"),
                ("Content-Encoding", "gzip"),
            ],
        )
        .await?;
        conn.write_all(APP_JS_GZ).await?;
        Ok(())
    }
}

impl Handler for HttpHandler {
    type Error<E>
        = Error<E>
    where
        E: Debug;

    async fn handle<T, const N: usize>(
        &self,
        _task_id: impl Display + Copy,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Self::Error<T::Error>>
    where
        T: Read + Write,
    {
        let headers = conn.headers()?;
        let method = headers.method;
        let path = headers.path;

        // Route dispatch
        match (method, path) {
            // GET endpoints
            (Method::Get, "/status") => self.handle_get_status(conn).await,
            (Method::Get, "/configuration") => self.handle_get_configuration(conn).await,
            (Method::Get, "/machine-definition") => self.handle_get_machine_definition(conn).await,
            (Method::Get, "/routines") => self.handle_get_routines(conn).await,

            // Schedule CRUD
            (Method::Post, "/schedules") => self.handle_post_schedule(conn).await,
            (Method::Put, p) if p.starts_with("/schedules/") => {
                if let Some(index) = Self::parse_path_index(p, "/schedules/") {
                    self.handle_put_schedule(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid schedule index").await
                }
            }
            (Method::Delete, p) if p.starts_with("/schedules/") => {
                if let Some(index) = Self::parse_path_index(p, "/schedules/") {
                    self.handle_delete_schedule(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid schedule index").await
                }
            }

            // Routine CRUD
            (Method::Post, p) if p.starts_with("/routines/") => {
                let remainder = p.strip_prefix("/routines/").unwrap_or("");
                self.handle_post_routine(conn, remainder).await
            }
            (Method::Put, p) if p.starts_with("/routines/") => {
                let remainder = p.strip_prefix("/routines/").unwrap_or("");
                let parts: Vec<&str> = remainder.split('/').collect();
                if parts.len() == 2 {
                    if let Ok(index) = parts[1].parse::<u32>() {
                        self.handle_put_routine(conn, parts[0], index).await
                    } else {
                        Self::send_bad_request(conn, "Invalid routine index").await
                    }
                } else {
                    Self::send_bad_request(conn, "Invalid path format").await
                }
            }
            (Method::Delete, p) if p.starts_with("/routines/") => {
                let remainder = p.strip_prefix("/routines/").unwrap_or("");
                let parts: Vec<&str> = remainder.split('/').collect();
                if parts.len() == 2 {
                    if let Ok(index) = parts[1].parse::<u32>() {
                        self.handle_delete_routine(conn, parts[0], index).await
                    } else {
                        Self::send_bad_request(conn, "Invalid routine index").await
                    }
                } else {
                    Self::send_bad_request(conn, "Invalid path format").await
                }
            }

            // Command endpoints
            (Method::Post, p) if p.starts_with("/command/run-routine/") => {
                let remainder = p.strip_prefix("/command/run-routine/").unwrap_or("");
                let parts: Vec<&str> = remainder.split('/').collect();
                if parts.len() == 2 {
                    if let Ok(index) = parts[1].parse::<u32>() {
                        self.handle_run_routine(conn, parts[0], index).await
                    } else {
                        Self::send_bad_request(conn, "Invalid routine index").await
                    }
                } else {
                    Self::send_bad_request(conn, "Invalid path format").await
                }
            }
            (Method::Post, "/command/cancel-routine") => self.handle_cancel_routine(conn).await,
            (Method::Post, p) if p.starts_with("/command/tare-group-scale/") => {
                if let Some(index) = Self::parse_path_u8_index(p, "/command/tare-group-scale/") {
                    self.handle_tare_group_scale(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid group index").await
                }
            }
            (Method::Post, p) if p.starts_with("/command/zero-calibrate-group-scale/") => {
                if let Some(index) =
                    Self::parse_path_u8_index(p, "/command/zero-calibrate-group-scale/")
                {
                    self.handle_zero_calibrate_group_scale(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid group index").await
                }
            }
            (Method::Post, p) if p.starts_with("/command/calibrate-group-scale-100g/") => {
                if let Some(index) =
                    Self::parse_path_u8_index(p, "/command/calibrate-group-scale-100g/")
                {
                    self.handle_calibrate_group_scale_100g(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid group index").await
                }
            }
            (Method::Post, p) if p.starts_with("/command/set-mode/") => {
                let mode_str = p.strip_prefix("/command/set-mode/").unwrap_or("");
                self.handle_set_mode(conn, mode_str).await
            }
            (Method::Post, "/command/set-boiler-control") => {
                self.handle_set_boiler_control(conn).await
            }
            (Method::Post, "/command/set-group-control") => {
                self.handle_set_group_control(conn).await
            }
            (Method::Post, "/command/set-pid-parameters") => {
                self.handle_set_pid_parameters(conn).await
            }
            (Method::Post, "/command/set-group-pump-configuration") => {
                self.handle_set_group_pump_configuration(conn).await
            }
            (Method::Post, "/command/set-water-tap-pump-configuration") => {
                self.handle_set_water_tap_pump_configuration(conn).await
            }
            (Method::Post, "/command/set-fill-pump-configuration") => {
                self.handle_set_fill_pump_configuration(conn).await
            }
            (Method::Post, "/command/set-steam-valve-openness") => {
                self.handle_set_steam_valve_openness(conn).await
            }
            (Method::Post, "/command/optimize-routine-storage") => {
                self.handle_optimize_routine_storage(conn).await
            }
            (Method::Post, "/command/optimize-schedule-storage") => {
                self.handle_optimize_schedule_storage(conn).await
            }
            (Method::Post, "/command/optimize-configuration-storage") => {
                self.handle_optimize_configuration_storage(conn).await
            }

            // Frontend static files
            (Method::Get, "/") | (Method::Get, "/index.html") => {
                Self::handle_index_html(conn).await
            }
            (Method::Get, p) if p.starts_with("/assets/") && p.ends_with(".js") => {
                Self::handle_js_gz(conn).await
            }

            // SPA fallback - serve index.html for unknown GET paths
            (Method::Get, _) => {
                log_info!("SPA fallback for: {}", path);
                Self::handle_index_html(conn).await
            }

            // 404 for non-GET methods on unknown paths
            _ => {
                defmt::warn!("Unknown endpoint: {} {}", defmt::Debug2Format(&method), path);
                Self::send_not_found(conn).await
            }
        }
    }
}

/// Cache update task - updates STATUS_CACHE and CONFIG_CACHE from subscribers
#[embassy_executor::task]
pub async fn cache_update_task(
    mut status_subscriber: ApplicationStatusSubscriber,
    mut config_subscriber: ApplicationConfigurationSubscriber,
) {
    log_info!("Cache update task started");

    loop {
        match select(
            status_subscriber.next_message_pure(),
            config_subscriber.next_message_pure(),
        )
        .await
        {
            Either::First(status) => {
                let mut cache = STATUS_CACHE.lock().await;
                *cache = Some(status);
            }
            Either::Second(config) => {
                let mut cache = CONFIG_CACHE.lock().await;
                *cache = Some(config);
            }
        }
    }
}

/// HTTP server task
#[embassy_executor::task]
pub async fn http_server_task(
    tcp_stack: &'static Tcp<'static>,
    command_sender: &'static MachineCommandSender,
) {
    log_info!("Starting HTTP server on port 80...");

    let mut server = DefaultServer::new();
    let handler = HttpHandler::new(command_sender);

    let bind_addr = SocketAddr::from(([0, 0, 0, 0], 80));

    match tcp_stack.bind(bind_addr).await {
        Ok(acceptor) => {
            log_info!("HTTP server bound to port 80");
            if let Err(e) = server.run(None, acceptor, handler).await {
                log_error!("HTTP server error: {:?}", e);
            }
        }
        Err(e) => {
            log_error!("Failed to bind HTTP server to port 80: {:?}", e);
        }
    }
}
