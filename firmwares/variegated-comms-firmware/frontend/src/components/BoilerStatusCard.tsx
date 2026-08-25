import { memo } from 'preact/compat';
import { Badge, Readout, ReadoutGroup, tokens, type StatusRole } from '@variegated-coffee/ui';
import { useMachine } from '../contexts/MachineContext';
import { BoilerStatus } from '../schemas/schemas';

interface BoilerStatusCardProps {
  index: number;
  status: BoilerStatus;
}

/**
 * The control mode as a status role.
 *
 * `Temperature` and `Pressure` are both "this boiler is being controlled" -- they differ in
 * *what* is being held, which the badge's own text says. They were previously green and
 * blue, which read as two different degrees of good. `Off` is not a fault either; it is
 * `idle`, and `Badge` renders a role-less badge in exactly that grey.
 */
function modeRole(mode: string): StatusRole | undefined {
  return mode === 'Off' ? undefined : 'ok';
}

const BoilerStatusCardComponent = ({ index, status }: BoilerStatusCardProps) => {
  const { getBoilerName, hasBoilerSensor } = useMachine();
  const name = getBoilerName(index);

  const output = status.output;

  return (
    <div
      style={{
        display: 'flex',
        flexDirection: 'column',
        gap: tokens.space.sm,
        padding: tokens.space.md,
        backgroundColor: tokens.color.surfaceRaised,
        border: `1px solid ${tokens.color.border}`,
        borderRadius: tokens.radius.md,
        flex: '1 1 300px',
        minWidth: '250px',
      }}
    >
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
        <h3 style={{ margin: 0, fontSize: '1.1rem', fontWeight: 600 }}>{name}</h3>
        <Badge role={modeRole(status.control_state.mode.type)}>
          {status.control_state.mode.type}
        </Badge>
      </div>

      <ReadoutGroup>
        {hasBoilerSensor(index, { type: 'Temperature' }) &&
          status.temperature !== null &&
          status.temperature !== undefined && (
            <Readout label="Temperature" value={status.temperature.toFixed(1)} unit="°C" />
          )}

        {hasBoilerSensor(index, { type: 'Pressure' }) &&
          status.pressure !== null &&
          status.pressure !== undefined && (
            <Readout label="Pressure" value={status.pressure.toFixed(2)} unit="bar" />
          )}

        {hasBoilerSensor(index, { type: 'WaterLevel' }) &&
          status.water_level !== null &&
          status.water_level !== undefined && (
            <Readout label="Water level" value={status.water_level.toFixed(1)} unit="%" />
          )}
      </ReadoutGroup>

      {/* The target, only for the quantity actually being controlled. */}
      {status.control_state.mode.type !== 'Off' && (
        <div style={{ paddingTop: tokens.space.sm, borderTop: `1px solid ${tokens.color.border}` }}>
          {status.control_state.mode.type === 'Temperature' ? (
            <Readout
              label="Target"
              value={status.control_state.values.target_temperature.toFixed(1)}
              unit="°C"
              emphasis
            />
          ) : (
            <Readout
              label="Target"
              value={status.control_state.values.target_pressure.toFixed(2)}
              unit="bar"
              emphasis
            />
          )}
        </div>
      )}

      <div
        style={{
          display: 'flex',
          flexDirection: 'column',
          gap: tokens.space.sm,
          paddingTop: tokens.space.sm,
          borderTop: `1px solid ${tokens.color.border}`,
        }}
      >
        {output.type === 'Off' && <Readout label="Output" value="Off" emphasis />}
        {output.type === 'FixedDutyCycle' && (
          <Readout label="Output" value={output.value.toFixed(1)} unit="% fixed" emphasis />
        )}
        {output.type === 'PidOutput' && (
          <>
            <Readout label="Output" value={output.value.out.toFixed(1)} unit="% PID" emphasis />

            {/* Loop internals. Kept, because they are how a boiler that is misbehaving gets
                diagnosed -- but visibly secondary to the figure above, which is the one a
                machine owner reads. */}
            <div
              style={{
                display: 'flex',
                flexDirection: 'column',
                gap: tokens.space.sm,
                padding: tokens.space.sm,
                backgroundColor: tokens.color.surfaceSunken,
                borderRadius: tokens.radius.sm,
              }}
            >
              <ReadoutGroup title="PID terms">
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: tokens.space.sm }}>
                  <Readout label="P" value={output.value.p.toFixed(2)} size="sm" />
                  <Readout label="I" value={output.value.i.toFixed(2)} size="sm" />
                  <Readout label="D" value={output.value.d.toFixed(2)} size="sm" />
                  <Readout label="Sum" value={output.value.out.toFixed(2)} size="sm" emphasis />
                </div>
              </ReadoutGroup>

              <ReadoutGroup title="Acting parameters">
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: tokens.space.sm }}>
                  <Readout label="Kp" value={output.value.acting_kp.toFixed(4)} size="sm" />
                  <Readout label="Ki" value={output.value.acting_ki.toFixed(4)} size="sm" />
                  <Readout label="Kd" value={output.value.acting_kd.toFixed(4)} size="sm" />
                </div>
              </ReadoutGroup>
            </div>
          </>
        )}
      </div>
    </div>
  );
};

export const BoilerStatusCard = memo(BoilerStatusCardComponent);
