import { memo } from 'preact/compat';
import { useState } from 'preact/hooks';
import {
  Alert,
  Badge,
  Button,
  Reading,
  ReadingGroup,
  tokens,
  useDialogs,
} from '@variegated-coffee/ui';
import { useMachine } from '../contexts/MachineContext';
import { GroupStatus } from '../schemas/schemas';
import { getWebSocketService } from '../services/websocket';

interface GroupStatusCardProps {
  index: number;
  status: GroupStatus;
}

/**
 * The three shot phases, using the design system's own phase vocabulary.
 *
 * These were `#ffc107`, `#fd7e14` and `#28a745` with a `22` alpha suffix -- an amber, an
 * orange and the same green the status badge uses, so "extracting" and "everything is
 * fine" were the same colour. `tokens.phase` already names exactly these three states,
 * because the shot charts shade the same bands behind their traces. Using them here means
 * a phase looks the same on the machine as it does on the chart of the shot it produced.
 */
const SHOT_PHASES: Record<string, { label: string; fill: string; description: string }> = {
  HeadspaceFill: {
    label: 'Headspace fill',
    fill: tokens.phase.headspaceFill,
    description: 'Filling headspace and wetting the puck',
  },
  Saturation: {
    label: 'Saturation',
    fill: tokens.phase.saturation,
    description: 'Puck saturating, pressure building',
  },
  PostFirstDrop: {
    label: 'Extracting',
    fill: tokens.phase.postFirstDrop,
    description: 'First drops detected, extracting',
  },
};

const GroupStatusCardComponent = ({ index, status }: GroupStatusCardProps) => {
  const { getGroupName, hasGroupSensor } = useMachine();
  const { confirm } = useDialogs();
  const name = getGroupName(index);
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);

  const showNotice = (message: string) => {
    setNotice(message);
    setTimeout(() => setNotice(null), 3000);
  };

  const showError = (message: string) => {
    setError(message);
    setTimeout(() => setError(null), 5000);
  };

  /**
   * Send a scale command, and say only what is actually known.
   *
   * These used to report "Scale tared successfully" the instant the command was queued.
   * Nothing here can know that: the command goes out over the websocket, and a tare on
   * these scales spans several measuring cycles before the reading settles at zero. The
   * wording now describes what happened -- the command was sent -- and leaves the reading
   * itself to say when it took effect, which it does, live, a few rows above.
   */
  const send = (action: (ws: NonNullable<ReturnType<typeof getWebSocketService>>) => void, sent: string) => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('Not connected to the machine');
      return;
    }
    action(ws);
    showNotice(sent);
  };

  const handleTareScale = () => {
    send((ws) => ws.tareGroupScale(index), 'Tare sent — watch the weight settle to zero');
  };

  /*
   * Both calibrations are confirmed, and the confirmation states the physical precondition
   * rather than asking "are you sure". They overwrite a stored calibration, and getting
   * one wrong means every shot weight after it is wrong -- quietly, and by an amount
   * nothing on screen would reveal.
   */
  const handleZeroCalibrateScale = () => {
    void confirm({
      title: 'Zero-calibrate this scale?',
      body: 'Take everything off the scale first. This replaces the stored zero point, and every weight after it is measured against the new one.',
      confirmLabel: 'Calibrate zero',
    }).then((ok) => {
      if (ok) send((ws) => ws.zeroCalibrateGroupScale(index), 'Zero calibration sent');
    });
  };

  const handleCalibrateScale100g = () => {
    void confirm({
      title: 'Calibrate this scale with 100 g?',
      body: 'Place a known 100 g weight on the scale first. This replaces the stored scale factor.',
      confirmLabel: 'Calibrate',
    }).then((ok) => {
      if (ok) send((ws) => ws.calibrateGroupScale100g(index), '100 g calibration sent');
    });
  };

  const shotPhase = status.current_brew?.shot_state
    ? SHOT_PHASES[status.current_brew.shot_state.type]
    : undefined;

  const formatBrewTime = (brew_time: { secs: bigint; nanos: number } | null | undefined) => {
    if (!brew_time) return '0.0';
    const totalMs = Number(brew_time.secs) * 1000 + brew_time.nanos / 1000000;
    return (totalMs / 1000).toFixed(1);
  };

  // `pump_output` is on the pump's own 0-255 scale, not a percentage -- the schema decodes
  // `HexadecimalDutyCycle` transparently to a `number`, so nothing here would have caught
  // the old `%` suffix having become wrong by a factor of 2.55. Both are shown: the raw
  // value is what the pump was given, and the percentage is what an operator set.
  const PUMP_FULL_SCALE = 255;
  const asPercent = (raw: number) => (raw * 100) / PUMP_FULL_SCALE;

  const pump = status.pump_output;

  return (
    <div
      style={{
        display: 'flex',
        flexDirection: 'column',
        gap: tokens.space.sm,
        padding: tokens.space.md,
        backgroundColor: tokens.color.surfaceRaised,
        // 1px in both states, with the emphasis carried by a ring rather than by a thicker
        // border -- a 2px border only while brewing shifted the card's contents by a pixel
        // at the start of every shot.
        border: `1px solid ${status.is_brewing ? tokens.color.ok : tokens.color.border}`,
        boxShadow: status.is_brewing ? `0 0 0 1px ${tokens.color.ok}` : undefined,
        borderRadius: tokens.radius.md,
        flex: '1 1 300px',
        minWidth: '250px',
      }}
    >
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
        <h3 style={{ margin: 0, fontSize: '1.1rem', fontWeight: 600 }}>{name}</h3>
        <Badge role={status.is_brewing ? 'ok' : undefined} numeric={status.is_brewing}>
          {status.is_brewing
            ? `Brewing ${formatBrewTime(status.current_brew?.brew_time)} s`
            : 'Idle'}
        </Badge>
      </div>

      <ReadingGroup>
        <Reading label="Mode" value={status.control_state.mode.type} />

        {hasGroupSensor(index, { type: 'Temperature' }) &&
          status.temperature !== null &&
          status.temperature !== undefined && (
            <Reading label="Temperature" value={status.temperature.toFixed(1)} unit="°C" />
          )}

        {hasGroupSensor(index, { type: 'Pressure' }) &&
          status.pressure !== null &&
          status.pressure !== undefined && (
            <Reading label="Pressure" value={status.pressure.toFixed(2)} unit="bar" />
          )}

        {hasGroupSensor(index, { type: 'InputFlowRate' }) &&
          status.input_flow_rate !== null &&
          status.input_flow_rate !== undefined && (
            <Reading label="Input flow" value={status.input_flow_rate.toFixed(1)} unit="mL/s" />
          )}

        {hasGroupSensor(index, { type: 'OutputFlowRate' }) &&
          status.output_flow_rate !== null &&
          status.output_flow_rate !== undefined && (
            <Reading label="Output flow" value={status.output_flow_rate.toFixed(1)} unit="mL/s" />
          )}

        {hasGroupSensor(index, { type: 'Weight' }) &&
          status.output_weight !== null &&
          status.output_weight !== undefined && (
            <Reading label="Output weight" value={status.output_weight.toFixed(1)} unit="g" />
          )}

        {/* Brew-sensor readings. Output temperature and extraction rate have no
            SensorCapability of their own to gate on, so they rely on the value
            being present, which it only is once a BrewSensor reports. */}
        {status.output_temperature !== null && status.output_temperature !== undefined && (
          <Reading label="Output temp" value={status.output_temperature.toFixed(1)} unit="°C" />
        )}

        {/* mS/cm, not µS/cm, and two decimals rather than none: espresso runs around
            1-3 mS/cm at the spout, so the old µS label was out by a factor of a thousand and
            `toFixed(0)` then rounded the whole useful range to "1", "2" or "3". The unit is
            pinned in `ECType`. */}
        {hasGroupSensor(index, { type: 'ElectricalConductivity' }) &&
          status.output_electrical_conductivity !== null &&
          status.output_electrical_conductivity !== undefined && (
            <Reading
              label="Conductivity"
              value={status.output_electrical_conductivity.toFixed(2)}
              unit="mS/cm"
            />
          )}

        {/* Not a percentage of anything. Extraction rate is conductivity times output flow;
            see `ExtractionRateType`. */}
        {status.extraction_rate !== null && status.extraction_rate !== undefined && (
          <Reading
            label="Extraction"
            value={status.extraction_rate.toFixed(2)}
            unit="mS·mL/cm·s"
          />
        )}
      </ReadingGroup>

      {status.is_brewing && (
        <div
          style={{
            display: 'flex',
            flexDirection: 'column',
            gap: tokens.space.sm,
            paddingTop: tokens.space.sm,
            borderTop: `1px solid ${tokens.color.border}`,
          }}
        >
          {shotPhase && (
            <div
              style={{
                padding: tokens.space.sm,
                backgroundColor: shotPhase.fill,
                borderRadius: tokens.radius.sm,
              }}
            >
              {/* The phase name carries the identity, as it does on the charts -- these
                  fills are deliberately too pale to be told apart by colour alone. */}
              <div style={{ fontWeight: 600, fontSize: '0.85rem', color: tokens.color.ink }}>
                {shotPhase.label}
              </div>
              <div style={{ fontSize: '0.75rem', color: tokens.color.inkMuted, marginTop: '0.125rem' }}>
                {shotPhase.description}
              </div>
            </div>
          )}

          <ReadingGroup title="Current shot">
            {status.current_brew?.brew_input_volume != null && (
              <Reading
                label="Input volume"
                value={status.current_brew.brew_input_volume.toFixed(1)}
                unit="mL"
                size="sm"
              />
            )}
            {status.current_brew?.output_volume != null && (
              <Reading
                label="Output volume"
                value={status.current_brew.output_volume.toFixed(1)}
                unit="mL"
                size="sm"
              />
            )}
            {status.current_brew?.extracted_solids != null && (
              <Reading
                label="Extracted solids"
                value={status.current_brew.extracted_solids.toFixed(1)}
                unit="g"
                size="sm"
              />
            )}
          </ReadingGroup>
        </div>
      )}

      {!status.is_brewing && status.previous_brew && (
        <div style={{ paddingTop: tokens.space.sm, borderTop: `1px solid ${tokens.color.border}` }}>
          <ReadingGroup title="Last shot">
            <Reading
              label="Time"
              value={formatBrewTime(status.previous_brew.brew_time)}
              unit="s"
              size="sm"
            />
            {status.previous_brew.output_weight !== null &&
              status.previous_brew.output_weight !== undefined && (
                <Reading
                  label="Weight"
                  value={status.previous_brew.output_weight.toFixed(1)}
                  unit="g"
                  size="sm"
                />
              )}
          </ReadingGroup>
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
        {pump.type === 'Off' && <Reading label="Pump output" value="Off" emphasis />}
        {pump.type === 'FixedDutyCycle' && (
          <Reading
            label="Pump output"
            value={`${pump.value}/255`}
            unit={`${asPercent(pump.value).toFixed(0)}%`}
            emphasis
          />
        )}
        {pump.type === 'PidOutput' && (
          <>
            <Reading
              label="Pump output"
              value={`${pump.value.out.toFixed(1)}/255`}
              unit={`${asPercent(pump.value.out).toFixed(0)}%`}
              emphasis
            />
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
              <ReadingGroup title="PID terms">
                {/* See BoilerStatusCard: a tight gap runs the next label into the previous
                    right-aligned number. */}
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: `${tokens.space.xs} ${tokens.space.md}` }}>
                  <Reading label="P" value={pump.value.p.toFixed(2)} size="sm" />
                  <Reading label="I" value={pump.value.i.toFixed(2)} size="sm" />
                  <Reading label="D" value={pump.value.d.toFixed(2)} size="sm" />
                  <Reading label="Sum" value={pump.value.out.toFixed(2)} size="sm" emphasis />
                </div>
              </ReadingGroup>
              <ReadingGroup title="Acting parameters">
                {/* See BoilerStatusCard: a tight gap runs the next label into the previous
                    right-aligned number. */}
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: `${tokens.space.xs} ${tokens.space.md}` }}>
                  <Reading label="Kp" value={pump.value.acting_kp.toFixed(4)} size="sm" />
                  <Reading label="Ki" value={pump.value.acting_ki.toFixed(4)} size="sm" />
                  <Reading label="Kd" value={pump.value.acting_kd.toFixed(4)} size="sm" />
                </div>
              </ReadingGroup>
            </div>
          </>
        )}
      </div>

      {hasGroupSensor(index, { type: 'Weight' }) && (
        <div
          style={{
            display: 'flex',
            flexDirection: 'column',
            gap: tokens.space.sm,
            paddingTop: tokens.space.sm,
            borderTop: `1px solid ${tokens.color.border}`,
          }}
        >
          <div style={{ font: `0.85rem ${tokens.font.sans}`, fontWeight: 500, color: tokens.color.inkMuted }}>
            Scale
          </div>

          {notice && <Alert role="ok">{notice}</Alert>}
          {error && <Alert role="danger">{error}</Alert>}

          {/* One primary. Taring is the thing you do between shots; the two calibrations
              are setup, done once, and were previously competing with it in orange and
              green as though all three were equally routine. */}
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: tokens.space.sm }}>
            <Button variant="primary" size="sm" onClick={() => void handleTareScale()}>
              Tare
            </Button>
            <Button variant="secondary" size="sm" onClick={() => void handleZeroCalibrateScale()}>
              Calibrate zero
            </Button>
            <Button variant="secondary" size="sm" onClick={() => void handleCalibrateScale100g()}>
              Calibrate 100 g
            </Button>
          </div>
        </div>
      )}
    </div>
  );
};

export const GroupStatusCard = memo(GroupStatusCardComponent);
