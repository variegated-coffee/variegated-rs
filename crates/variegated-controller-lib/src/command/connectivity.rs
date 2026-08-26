//! Wi-Fi credentials, the provisioning window, the shot-upload settings and the timezone.
//!
//! Four small stores that both machines keep, all edited the same way and all written twice.
//! Each function here reports whether the value actually changed, because the
//! compare-before-save is load-bearing rather than an optimisation: the settings panel posts
//! on every save whether or not anything was edited, and the comms processor re-sends
//! credentials on every reconnect. Writing unconditionally would put a flash erase on both
//! paths.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Sender;
use variegated_controller_types::shot_upload::{ShotUploadConfig, ShotUploadSettings};
use variegated_controller_types::timezone::TimezoneSetting;
use variegated_controller_types::wifi::{StoredWifiCredentials, WifiCredentials};
use variegated_log::{log_info, log_warn};

/// Whether the caller should write the value to its store.
#[must_use = "a setting that is not persisted is lost at the next power cycle"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Persist(bool);

impl Persist {
    /// The value changed and should be written.
    pub const YES: Self = Self(true);
    /// Nothing changed, so nothing should be written.
    pub const NO: Self = Self(false);

    /// Whether a write is wanted.
    pub const fn wanted(self) -> bool {
        self.0
    }
}

/// `SetWifiCredentials`.
///
/// **Persisted without validation, and that is correct rather than lax**: these arrive only
/// from `WifiCredentialsProvisioned`, which the comms processor sends *after* its radio has
/// associated using them. Nothing here could check them better than that.
pub fn set_wifi_credentials(
    stored: &mut StoredWifiCredentials,
    credentials: WifiCredentials,
) -> Persist {
    let incoming = StoredWifiCredentials(Some(credentials));
    if *stored == incoming {
        return Persist::NO;
    }
    *stored = incoming;
    log_info!("Stored new Wi-Fi credentials");
    Persist::YES
}

/// `SetShotUploadSettings` -- the settings UI's edit.
///
/// Merged rather than replacing, because the browser is never sent the token and so cannot
/// send it back; see `ShotUploadTokenUpdate` for the three-way answer that carries.
pub fn apply_shot_upload_settings(
    config: &mut ShotUploadConfig,
    settings: ShotUploadSettings,
) -> Persist {
    let mut updated = config.clone();
    updated.apply(settings);
    if *config == updated {
        return Persist::NO;
    }
    *config = updated;
    // Never the token, and not even its length -- see the type's `Debug`.
    log_info!(
        "Shot upload settings updated: endpoint {}, token {}, uploads {}",
        if config.endpoint.is_some() { "set" } else { "cleared" },
        if config.token.is_some() { "set" } else { "cleared" },
        if config.enabled { "enabled" } else { "disabled" }
    );
    Persist::YES
}

/// `SetShotUploadConfig` -- the CLI's full replace.
///
/// **Persisted without validation, for a different reason than the Wi-Fi arm above**: this
/// processor *could* parse the URL, but it is not the one that uses it. The comms processor
/// parses it at upload time and reports what it found, and two parsers on one string with
/// nothing keeping them in agreement is worse than one.
pub fn set_shot_upload_config(
    config: &mut ShotUploadConfig,
    incoming: ShotUploadConfig,
) -> Persist {
    if *config == incoming {
        return Persist::NO;
    }
    *config = incoming;
    log_info!(
        "Stored shot upload config: endpoint {}, token {}",
        if config.endpoint.is_some() { "set" } else { "cleared" },
        if config.token.is_some() { "set" } else { "cleared" }
    );
    Persist::YES
}

/// `OpenWifiProvisioningWindow`.
///
/// Refused while the machine is busy on the same grounds as a discovery scan: minutes of
/// connectable advertising share one antenna with Wi-Fi and with the live links to the
/// scales, and this is the only processor that knows coffee is being made. `busy` is the
/// caller's to decide, as it is for a scan.
pub fn open_provisioning_window<M: RawMutex>(
    sender: Option<Sender<'_, M, u32, 2>>,
    duration_ms: u32,
    busy: bool,
) {
    if busy {
        log_warn!("Refusing to open the Wi-Fi provisioning window: machine is busy");
        return;
    }
    match sender {
        Some(sender) => {
            if sender.try_send(duration_ms).is_err() {
                log_warn!("Failed to forward the provisioning window request: channel full");
            }
        }
        None => log_warn!("This machine has no Wi-Fi provisioning path"),
    }
}

/// `CloseWifiProvisioningWindow`.
///
/// Zero means close, on the same channel as the open -- so a close cannot overtake the open
/// it was meant to cancel.
pub fn close_provisioning_window<M: RawMutex>(sender: Option<Sender<'_, M, u32, 2>>) {
    if let Some(sender) = sender {
        let _ = sender.try_send(0);
    }
}

/// `SetTimezone`.
///
/// Applied to the `TimeKeeper` *before* it is stored, so a zone the keeper refuses does not
/// leave flash claiming a zone the machine is not keeping time in. A name this firmware's
/// trimmed database does not contain is refused and logged, and the machine keeps the zone it
/// had.
pub fn set_timezone(stored: &mut TimezoneSetting, setting: TimezoneSetting) -> Persist {
    let Some(zone) = variegated_timekeeping::TimeZoneWrapper::from_iana_name(setting.as_str())
    else {
        log_warn!("Refusing unknown timezone: {}", setting.as_str());
        return Persist::NO;
    };

    if let Err(e) = variegated_timekeeping::TimeKeeper::set_timezone(zone) {
        log_warn!("Failed to apply timezone: {:?}", e);
        return Persist::NO;
    }

    log_info!("Timezone set to {}", setting.as_str());
    *stored = setting;
    Persist::YES
}

#[cfg(test)]
mod tests {
    use super::*;
    use variegated_controller_types::shot_upload::{ShotUploadKeyUpdate, ShotUploadTokenUpdate};

    fn token(value: &str) -> heapless::String<{ variegated_controller_types::shot_upload::SHOT_UPLOAD_TOKEN_LEN }> {
        heapless::String::try_from(value).expect("the test tokens are short enough")
    }

    fn endpoint(value: &str) -> heapless::String<{ variegated_controller_types::shot_upload::SHOT_UPLOAD_ENDPOINT_LEN }> {
        heapless::String::try_from(value).expect("the test endpoints are short enough")
    }

    fn settings_keeping_the_token(url: &str) -> ShotUploadSettings {
        ShotUploadSettings {
            endpoint: Some(endpoint(url)),
            enabled: true,
            token: ShotUploadTokenUpdate::Keep,
            server_key: None,
            device_key: ShotUploadKeyUpdate::Keep,
        }
    }

    /// An edit that keeps the token preserves the stored one.
    ///
    /// The browser is never sent the token, so every endpoint edit arrives as `Keep`. If that
    /// cleared it, changing the URL would silently de-provision the machine.
    #[test]
    fn an_endpoint_edit_preserves_the_stored_token() {
        let mut config = ShotUploadConfig {
            endpoint: Some(endpoint("https://example.org/a")),
            token: Some(token("secret")),
            enabled: true,
            ..ShotUploadConfig::default()
        };

        let persist = apply_shot_upload_settings(&mut config, settings_keeping_the_token("https://example.org/b"));

        assert_eq!(persist, Persist::YES);
        assert_eq!(config.token, Some(token("secret")), "the token was lost");
        assert_eq!(config.endpoint, Some(endpoint("https://example.org/b")));
    }

    /// Re-posting the same settings changes nothing and asks for no write.
    ///
    /// The panel posts on every save whether or not anything was edited, so without this a
    /// flash erase would run every time someone opened the page and pressed save.
    #[test]
    fn an_unchanged_settings_post_does_not_persist() {
        let mut config = ShotUploadConfig {
            endpoint: Some(endpoint("https://example.org/a")),
            token: Some(token("secret")),
            enabled: true,
            ..ShotUploadConfig::default()
        };

        let persist = apply_shot_upload_settings(&mut config, settings_keeping_the_token("https://example.org/a"));

        assert_eq!(persist, Persist::NO);
    }

    /// Clearing the token is distinct from keeping it.
    #[test]
    fn clearing_the_token_is_not_keeping_it() {
        let mut config = ShotUploadConfig {
            endpoint: Some(endpoint("https://example.org/a")),
            token: Some(token("secret")),
            enabled: true,
            ..ShotUploadConfig::default()
        };

        let mut settings = settings_keeping_the_token("https://example.org/a");
        settings.token = ShotUploadTokenUpdate::Clear;
        let persist = apply_shot_upload_settings(&mut config, settings);

        assert_eq!(persist, Persist::YES);
        assert_eq!(config.token, None);
    }

    /// The same credentials arriving again do not cost a flash write.
    ///
    /// The comms processor re-sends these on every reconnect.
    #[test]
    fn repeated_wifi_credentials_do_not_persist_twice() {
        let credentials = WifiCredentials::default();
        let mut stored = StoredWifiCredentials(None);

        assert_eq!(set_wifi_credentials(&mut stored, credentials.clone()), Persist::YES);
        assert_eq!(set_wifi_credentials(&mut stored, credentials), Persist::NO);
    }

    /// A full-replace config that matches what is stored does not persist either.
    #[test]
    fn an_unchanged_upload_config_does_not_persist() {
        let mut config = ShotUploadConfig {
            endpoint: Some(endpoint("https://example.org/a")),
            ..ShotUploadConfig::default()
        };
        let same = config.clone();

        assert_eq!(set_shot_upload_config(&mut config, same), Persist::NO);
    }
}
