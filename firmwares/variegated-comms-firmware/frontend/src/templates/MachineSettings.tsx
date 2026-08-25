import { tokens } from '@variegated-coffee/ui';
import { ShotUploadPanel } from '../components/ShotUploadPanel';
import { BluetoothPanel } from '../components/BluetoothPanel';
import { ShotLogPanel } from '../components/ShotLogPanel';
import { TimezonePanel } from '../components/TimezonePanel';
import { ScheduleBuilder } from '../components/ScheduleBuilder';
import { RoutineBuilder } from '../components/routine/RoutineBuilder';
import { MachineDefinition, Status, Configuration, RoutineSummaryStorage } from '../schemas/schemas';

/**
 * The stack of setup panels below the dashboard.
 *
 * The second of the two templates, and the one that shows what a *panel* is: a card, one
 * per concern, each owning its own errors and empty states. The card style lives here
 * rather than in each `<section>`, which is how three of them came to have a slightly
 * different shadow.
 *
 * # Two pairings that are deliberate
 *
 * The timezone sits **in the same card as the schedules**, because a schedule time means
 * nothing without knowing which clock it is on.
 *
 * The shot log sits above the schedules and below Bluetooth, because the scale it takes
 * doses from is a Bluetooth peripheral — the thing you configure immediately before you
 * need it.
 */
export interface MachineSettingsProps {
  machineDefinition: MachineDefinition | null;
  status: Status;
  configuration: Configuration;
  routines: RoutineSummaryStorage | null;
  connected: boolean;
}

/**
 * The card each panel sits in.
 *
 * One object rather than the same properties written out at each `<section>`.
 */
const panelStyle = {
  marginTop: tokens.space.lg,
  background: tokens.color.surfaceRaised,
  padding: tokens.space.lg,
  border: `1px solid ${tokens.color.border}`,
  borderRadius: tokens.radius.md,
};

export function MachineSettings({
  machineDefinition,
  status,
  configuration,
  routines,
  connected,
}: MachineSettingsProps) {
  return (
    <>
      <section style={panelStyle}>
        <ShotUploadPanel shotUpload={configuration?.shot_upload} />
      </section>

      <section style={panelStyle}>
        <BluetoothPanel
          associations={configuration?.bluetooth_peripherals || []}
          scan={status.bluetooth}
          // The connection map is keyed by peripheral id and only exists once the comms
          // processor has reported in, so an absent entry reads as "not connected"
          // rather than as an error.
          connected={
            new Map(
              Array.from(status.comms_status?.peripheral_connection_status ?? []).map(
                ([id, wireless]) => [id, wireless.connected]
              )
            )
          }
        />
      </section>

      <section style={panelStyle}>
        <ShotLogPanel
          connected={connected}
          pending={status.pending_shot_annotations}
          sdCardPresent={status.sd_card_present}
          // The groups the machine actually has, rather than a hardcoded `[0]`: the
          // dose buttons address a group scale by index, and a two-group machine needs
          // two buttons.
          groupIndices={Array.from(status.group_statuses.keys())}
        />
      </section>

      {/* Schedules, and the timezone they fire on -- together, because a schedule time
          means nothing without knowing which clock it is on. */}
      <section style={panelStyle}>
        <TimezonePanel timezone={configuration?.timezone} />
        <ScheduleBuilder schedules={configuration?.schedules || []} />
      </section>

      {routines && (
        <section style={{ marginTop: tokens.space.lg }}>
          <RoutineBuilder
            routines={routines}
            machineDefinition={machineDefinition}
            peripheralStatus={status?.peripheral_status ?? null}
          />
        </section>
      )}
    </>
  );
}
