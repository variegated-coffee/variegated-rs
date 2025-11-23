import { memo } from 'preact/compat';
import { useState } from 'preact/hooks';
import { RoutineExecutionStatus, Status, RoutineStorage, RoutineCommand, RoutineExit } from '../schemas/schemas';
import { useMachine } from '../contexts/MachineContext';
import { formatCommand } from '../utils/commandFormatter';
import { formatExitCondition, formatExitAction } from '../utils/exitConditionFormatter';
import { getRoutineFromIndex, getRoutineIndexLabel } from '../utils/routineHelpers';
import { getWebSocketService } from '../services/websocket';

interface RoutineExecutionCardProps {
  execution: RoutineExecutionStatus;
  routines: RoutineStorage;
  status: Status;
}

const RoutineExecutionCardComponent = ({ execution: executionProp, routines, status }: RoutineExecutionCardProps) => {
  // Type assertion needed: Postcard's InferType fails on RoutineExecutionStatus type
  const execution = executionProp;

  const { getBoilerName, getGroupName } = useMachine();
  const [detailsExpanded, setDetailsExpanded] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);

  const showSuccess = (message: string) => {
    setSuccessMessage(message);
    setTimeout(() => setSuccessMessage(null), 3000);
  };

  const showError = (message: string) => {
    setError(message);
    setTimeout(() => setError(null), 5000);
  };

  const handleCancel = () => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }
    ws.cancelRoutine();
    showSuccess('Routine cancelled successfully');
  };

  // Get the routine being executed
  const routine = getRoutineFromIndex(routines, execution.routine_index);
  const routineLabel = getRoutineIndexLabel(execution.routine_index);

  if (!routine) {
    return (
      <div
        style={{
          padding: '0.75rem 1rem',
          backgroundColor: '#d1ecf1',
          border: '1px solid #bee5eb',
          borderRadius: '6px',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center'
        }}
      >
        <span style={{ fontWeight: '500' }}>Routine {routineLabel} Running (routine not found)</span>
      </div>
    );
  }

  // Get current step info
  const currentStep = execution.current_step !== null && execution.current_step !== undefined
    ? routine.steps[execution.current_step]
    : null;
  const totalSteps = routine.steps.length;
  const stepNumber = (execution.current_step ?? 0) + 1;

  // Format times
  const formatDuration = (duration: { secs: bigint; nanos: number } | null | undefined) => {
    if (!duration) return '0.0s';
    const totalSecs = Number(duration.secs) + duration.nanos / 1000000000;
    return `${totalSecs.toFixed(1)}s`;
  };

  const stepElapsedTime = formatDuration(execution.step_elapsed_time);
  const totalElapsedTime = formatDuration(execution.total_elapsed_time);

  // Calculate progress percentage
  const progressPercent = (stepNumber / totalSteps) * 100;

  // Format exit conditions in detailed format for Details section
  const formatDetailedExitConditions = () => {
    if (!currentStep || !currentStep.exits || currentStep.exits.length === 0) {
      return 'No exit conditions';
    }

    return currentStep.exits.map((exit: RoutineExit, idx: number) => {
      const conditionText = formatExitCondition(exit, getBoilerName, getGroupName);
      const description = exit.description || conditionText;
      const action = formatExitAction(exit);

      return (
        <div key={idx} style={{ fontSize: '0.85rem', marginBottom: '0.25rem' }}>
          <span style={{ color: '#0066cc', fontWeight: '500' }}>•</span> {description}
          {conditionText !== description && (
            <span style={{ color: '#888', marginLeft: '0.5rem' }}>({conditionText})</span>
          )}
          <span style={{ color: '#666', marginLeft: '0.5rem', fontSize: '0.8rem' }}>→ {action}</span>
        </div>
      );
    });
  };

  // Format exit conditions with current values in simplified format
  const formatExitConditions = () => {
    if (!currentStep || !currentStep.exits || currentStep.exits.length === 0) {
      return null;
    }

    return currentStep.exits.map((exit: RoutineExit, idx: number) => {
      let label = '';
      let currentVal = '';
      let targetVal = '';
      let unit = '';

      if (exit.condition.type === 'Always' || exit.condition.type === 'Never') {
        return (
          <div key={idx} style={{ fontSize: '0.9rem', color: '#155724' }}>
            {exit.description || exit.condition.type}
          </div>
        );
      }

      if (exit.condition.type === 'After') {
        const condValue = exit.condition.value;
        const value = condValue.type === 'Static' ? condValue.value : 0;
        const stepElapsed = execution.step_elapsed_time;
        const currentSeconds = stepElapsed ? (Number(stepElapsed.secs) + stepElapsed.nanos / 1000000000) : 0;
        label = exit.description || 'Time';
        currentVal = currentSeconds.toFixed(1);
        targetVal = value.toFixed(1);
        unit = 's';
      } else if (exit.condition.type === 'AfterDurationRelativeToStart') {
        const condValue = exit.condition.value;
        const value = condValue.type === 'Static' ? condValue.value : 0;
        const totalElapsed = execution.total_elapsed_time;
        const currentSeconds = totalElapsed ? (Number(totalElapsed.secs) + totalElapsed.nanos / 1000000000) : 0;
        label = exit.description || 'Total Time';
        currentVal = currentSeconds.toFixed(1);
        targetVal = value.toFixed(1);
        unit = 's';
      } else if (exit.condition.type === 'StateConditionMet') {
        const stateCondition = exit.condition.value;

        if (stateCondition.type === 'InputVolumeAboveRelativeToStart') {
          const [groupIdx, targetValue] = stateCondition.value;
          const value = targetValue.type === 'Static' ? targetValue.value : 0;
          const groupKey = Array.from(status.group_statuses.keys())[groupIdx];
          const groupStatus = status.group_statuses.get(groupKey);
          label = exit.description || 'Input Volume';
          currentVal = (groupStatus?.brew_input_volume ?? 0).toFixed(1);
          targetVal = value.toFixed(1);
          unit = 'mL';
        } else if (stateCondition.type === 'BoilerTemperatureAbove' || stateCondition.type === 'BoilerTemperatureBelow') {
          const [boilerIdx, targetValue] = stateCondition.value;
          const value = targetValue.type === 'Static' ? targetValue.value : 0;
          const boilerKey = Array.from(status.boiler_statuses.keys())[boilerIdx];
          const boilerStatus = status.boiler_statuses.get(boilerKey);
          label = exit.description || 'Boiler Temp';
          currentVal = (boilerStatus?.temperature ?? 0).toFixed(1);
          targetVal = value.toFixed(1);
          unit = '°C';
        } else if (stateCondition.type === 'BoilerPressureAbove' || stateCondition.type === 'BoilerPressureBelow') {
          const [boilerIdx, targetValue] = stateCondition.value;
          const value = targetValue.type === 'Static' ? targetValue.value : 0;
          const boilerKey = Array.from(status.boiler_statuses.keys())[boilerIdx];
          const boilerStatus = status.boiler_statuses.get(boilerKey);
          label = exit.description || 'Boiler Pressure';
          currentVal = (boilerStatus?.pressure ?? 0).toFixed(2);
          targetVal = value.toFixed(2);
          unit = 'bar';
        } else if (stateCondition.type === 'GroupPressureAbove' || stateCondition.type === 'GroupPressureBelow') {
          const [groupIdx, targetValue] = stateCondition.value;
          const value = targetValue.type === 'Static' ? targetValue.value : 0;
          const groupKey = Array.from(status.group_statuses.keys())[groupIdx];
          const groupStatus = status.group_statuses.get(groupKey);
          label = exit.description || 'Group Pressure';
          currentVal = (groupStatus?.pressure ?? 0).toFixed(2);
          targetVal = value.toFixed(2);
          unit = 'bar';
        } else if (stateCondition.type === 'OutputWeightAbove' || stateCondition.type === 'OutputWeightBelow') {
          const [groupIdx, targetValue] = stateCondition.value;
          const value = targetValue.type === 'Static' ? targetValue.value : 0;
          const groupKey = Array.from(status.group_statuses.keys())[groupIdx];
          const groupStatus = status.group_statuses.get(groupKey);
          label = exit.description || 'Output Weight';
          currentVal = (groupStatus?.output_weight ?? 0).toFixed(1);
          targetVal = value.toFixed(1);
          unit = 'g';
        } else if (stateCondition.type === 'Brewing' || stateCondition.type === 'NotBrewing') {
          const groupIdx = stateCondition.value;
          const groupKey = Array.from(status.group_statuses.keys())[groupIdx];
          const groupStatus = status.group_statuses.get(groupKey);
          return (
            <div key={idx} style={{ fontSize: '0.9rem', color: '#155724' }}>
              {exit.description || `Group ${groupIdx} ${stateCondition.type === 'Brewing' ? 'brewing' : 'not brewing'}`}:{' '}
              <span style={{ fontWeight: '600' }}>{groupStatus?.is_brewing ? 'Yes' : 'No'}</span>
            </div>
          );
        } else {
          return (
            <div key={idx} style={{ fontSize: '0.9rem', color: '#155724' }}>
              {exit.description || stateCondition.type}
            </div>
          );
        }
      }

      if (!label) return null;

      return (
        <div key={idx} style={{ fontSize: '0.9rem', color: '#155724' }}>
          {label}:{' '}
          <span style={{ fontWeight: '600' }}>
            {currentVal} {unit}
          </span>
          {' → '}
          <span style={{ fontWeight: '600' }}>
            {targetVal} {unit}
          </span>
        </div>
      );
    });
  };

  return (
    <div
      style={{
        padding: '1rem',
        backgroundColor: '#d4edda',
        border: '2px solid #28a745',
        borderRadius: '8px',
        marginBottom: '1rem'
      }}
    >
      {/* Header */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.75rem' }}>
        <div>
          <h3 style={{ margin: 0, fontSize: '1.1rem', fontWeight: '600', color: '#155724' }}>
            {routine.name}
          </h3>
          <div style={{ fontSize: '0.9rem', color: '#155724', marginTop: '0.25rem' }}>
            Step {stepNumber} of {totalSteps}
            {currentStep?.description && `: ${currentStep.description}`}
          </div>
        </div>
        <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
          <span
            style={{
              padding: '0.35rem 0.85rem',
              backgroundColor: '#28a745',
              color: 'white',
              borderRadius: '12px',
              fontSize: '0.8rem',
              fontWeight: '600',
              textTransform: 'uppercase'
            }}
          >
            Running
          </span>
          <button
            onClick={() => void handleCancel()}
            style={{
              padding: '0.35rem 0.85rem',
              backgroundColor: '#dc3545',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer',
              fontSize: '0.8rem',
              fontWeight: '500'
            }}
          >
            Cancel
          </button>
        </div>
      </div>

      {/* Error Message */}
      {error && (
        <div style={{
          fontSize: '0.9rem',
          color: '#721c24',
          marginBottom: '0.75rem',
          padding: '0.5rem',
          backgroundColor: '#f8d7da',
          borderRadius: '4px',
          border: '1px solid #f5c6cb'
        }}>
          ❌ {error}
        </div>
      )}

      {/* Success Message */}
      {successMessage && (
        <div style={{
          fontSize: '0.9rem',
          color: '#155724',
          marginBottom: '0.75rem',
          padding: '0.5rem',
          backgroundColor: '#d4edda',
          borderRadius: '4px',
          border: '1px solid #c3e6cb'
        }}>
          ✅ {successMessage}
        </div>
      )}

      {/* Progress Bar */}
      <div
        style={{
          width: '100%',
          height: '8px',
          backgroundColor: '#c3e6cb',
          borderRadius: '4px',
          overflow: 'hidden',
          marginBottom: '0.75rem'
        }}
      >
        <div
          style={{
            width: `${progressPercent}%`,
            height: '100%',
            backgroundColor: '#28a745',
            transition: 'width 0.3s ease'
          }}
        />
      </div>

      {/* Timers */}
      <div style={{ display: 'flex', gap: '1.5rem', marginBottom: '0.75rem', fontSize: '0.9rem' }}>
        <div>
          <span style={{ color: '#155724', fontWeight: '500' }}>Step Time:</span>{' '}
          <span style={{ fontWeight: '600' }}>{stepElapsedTime}</span>
        </div>
        <div>
          <span style={{ color: '#155724', fontWeight: '500' }}>Total Time:</span>{' '}
          <span style={{ fontWeight: '600' }}>{totalElapsedTime}</span>
        </div>
      </div>

      {/* Exit Conditions (Simplified) */}
      <div style={{ marginBottom: '0.75rem' }}>
        {formatExitConditions()}
      </div>

      {/* Details Toggle */}
      <button
        onClick={() => setDetailsExpanded(!detailsExpanded)}
        style={{
          background: 'none',
          border: 'none',
          color: '#155724',
          cursor: 'pointer',
          fontSize: '0.85rem',
          fontWeight: '500',
          padding: '0.25rem 0',
          display: 'flex',
          alignItems: 'center',
          gap: '0.25rem'
        }}
      >
        <span style={{ fontSize: '0.7rem' }}>{detailsExpanded ? '▼' : '▶'}</span>
        Details
      </button>

      {/* Expanded Details */}
      {detailsExpanded && (
        <div
          style={{
            marginTop: '0.75rem',
            padding: '0.75rem',
            backgroundColor: '#fff',
            borderRadius: '6px',
            border: '1px solid #c3e6cb'
          }}
        >
          {/* Entry Commands */}
          <div style={{ marginBottom: '0.75rem' }}>
            <div style={{ fontSize: '0.85rem', fontWeight: '600', color: '#155724', marginBottom: '0.25rem' }}>
              Entry Commands:
            </div>
            <div style={{ fontSize: '0.85rem', color: '#333' }}>
              {currentStep?.entry_command && currentStep.entry_command.length > 0 ? (
                currentStep.entry_command.length === 1 ? (
                  formatCommand(currentStep.entry_command[0], getBoilerName, getGroupName)
                ) : (
                  <ol style={{ margin: 0, paddingLeft: '1.5rem' }}>
                    {currentStep.entry_command.map((cmd: RoutineCommand, i: number) => (
                      <li key={i}>{formatCommand(cmd, getBoilerName, getGroupName)}</li>
                    ))}
                  </ol>
                )
              ) : (
                'No entry commands'
              )}
            </div>
          </div>

          {/* Exit Conditions (Detailed) */}
          <div>
            <div style={{ fontSize: '0.85rem', fontWeight: '600', color: '#155724', marginBottom: '0.25rem' }}>
              Exit Conditions:
            </div>
            {formatDetailedExitConditions()}
          </div>

          {/* Parameters (if any) */}
          {execution.resolved_parameters.size > 0 && (
            <div style={{ marginTop: '0.75rem' }}>
              <div style={{ fontSize: '0.85rem', fontWeight: '600', color: '#155724', marginBottom: '0.25rem' }}>
                Parameters:
              </div>
              {Array.from(execution.resolved_parameters.entries()).map(([key, value]) => (
                <div key={key} style={{ fontSize: '0.85rem', color: '#333' }}>
                  {key}: {value}
                </div>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export const RoutineExecutionCard = memo(RoutineExecutionCardComponent);
