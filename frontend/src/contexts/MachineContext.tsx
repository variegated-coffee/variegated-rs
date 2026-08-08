import { createContext } from 'preact';
import { useContext, useCallback, useMemo } from 'preact/hooks';
import {
  MachineDefinition,
  SensorCapability,
  ControlModeCapability,
  BoilerEntry,
  GroupEntry,
  WaterTapEntry,
  SteamWandEntry,
  PeripheralEntry
} from '../schemas/schemas';

interface MachineContextType {
  machineDefinition: MachineDefinition | null;
  getBoilerName: (index: number) => string;
  getGroupName: (index: number) => string;
  getWaterTapName: (index: number) => string;
  getSteamWandName: (index: number) => string;
  getTankName: (index: number) => string;
  getBoilerCount: () => number;
  getGroupCount: () => number;
  getWaterTapCount: () => number;
  getSteamWandCount: () => number;
  hasBoilerSensor: (index: number, sensorType: SensorCapability) => boolean;
  hasGroupSensor: (index: number, sensorType: SensorCapability) => boolean;
  getBoilerControlModes: (index: number) => ControlModeCapability[];
  getGroupControlModes: (index: number) => ControlModeCapability[];
  getBoilerEntries: () => BoilerEntry[];
  getGroupEntries: () => GroupEntry[];
  getWaterTapEntries: () => WaterTapEntry[];
  getSteamWandEntries: () => SteamWandEntry[];
  getCommsPeripheralEntries: () => PeripheralEntry[];
}

const MachineContext = createContext<MachineContextType | undefined>(undefined);

export function MachineProvider({
  machineDefinition,
  children
}: {
  machineDefinition: MachineDefinition | null;
  children: preact.ComponentChildren;
}) {
  const getBoilerName = useCallback((index: number): string => {
    if (!machineDefinition) return `Boiler ${index}`;
    const boiler = machineDefinition.boilers.get(index);
    return boiler ? boiler.name : `Boiler ${index}`;
  }, [machineDefinition]);

  const getGroupName = useCallback((index: number): string => {
    if (!machineDefinition) return `Group ${index}`;
    const group = machineDefinition.groups.get(index);
    return group ? group.name : `Group ${index}`;
  }, [machineDefinition]);

  const getWaterTapName = useCallback((index: number): string => {
    if (!machineDefinition) return `Water Tap ${index}`;
    const waterTap = machineDefinition.water_taps.get(index);
    return waterTap ? waterTap.name : `Water Tap ${index}`;
  }, [machineDefinition]);

  const getSteamWandName = useCallback((index: number): string => {
    if (!machineDefinition) return `Steam Wand ${index}`;
    const steamWand = machineDefinition.steam_wands.get(index);
    return steamWand ? steamWand.name : `Steam Wand ${index}`;
  }, [machineDefinition]);

  const getTankName = useCallback((index: number): string => {
    if (!machineDefinition) return `Tank ${index}`;
    const tank = machineDefinition.tanks.get(index);
    return tank ? tank.name : `Tank ${index}`;
  }, [machineDefinition]);

  const getBoilerCount = useCallback((): number => {
    return machineDefinition ? machineDefinition.boilers.size : 0;
  }, [machineDefinition]);

  const getGroupCount = useCallback((): number => {
    return machineDefinition ? machineDefinition.groups.size : 0;
  }, [machineDefinition]);

  const getWaterTapCount = useCallback((): number => {
    return machineDefinition ? machineDefinition.water_taps.size : 0;
  }, [machineDefinition]);

  const getSteamWandCount = useCallback((): number => {
    return machineDefinition ? machineDefinition.steam_wands.size : 0;
  }, [machineDefinition]);

  const hasBoilerSensor = useCallback((index: number, sensorType: SensorCapability): boolean => {
    if (!machineDefinition) return false;
    const boiler = machineDefinition.boilers.get(index);
    return boiler ? boiler.sensors.some(s => s.type === sensorType.type) : false;
  }, [machineDefinition]);

  const hasGroupSensor = useCallback((index: number, sensorType: SensorCapability): boolean => {
    if (!machineDefinition) return false;
    const group = machineDefinition.groups.get(index);
    return group ? group.sensors.some(s => s.type === sensorType.type) : false;
  }, [machineDefinition]);

  const getBoilerControlModes = useCallback((index: number): ControlModeCapability[] => {
    if (!machineDefinition) return [];
    const boiler = machineDefinition.boilers.get(index);
    return boiler ? boiler.control_modes : [];
  }, [machineDefinition]);

  const getGroupControlModes = useCallback((index: number): ControlModeCapability[] => {
    if (!machineDefinition) return [];
    const group = machineDefinition.groups.get(index);
    return group ? group.control_modes : [];
  }, [machineDefinition]);

  const getBoilerEntries = useCallback(() => {
    return machineDefinition ? Array.from(machineDefinition.boilers.entries()) : [];
  }, [machineDefinition]);

  const getGroupEntries = useCallback(() => {
    return machineDefinition ? Array.from(machineDefinition.groups.entries()) : [];
  }, [machineDefinition]);

  const getWaterTapEntries = useCallback(() => {
    return machineDefinition ? Array.from(machineDefinition.water_taps.entries()) : [];
  }, [machineDefinition]);

  const getSteamWandEntries = useCallback(() => {
    return machineDefinition ? Array.from(machineDefinition.steam_wands.entries()) : [];
  }, [machineDefinition]);

  /**
   * Peripherals the comms processor owns, which are the only ones a Bluetooth
   * association can name.
   *
   * The machine definition is where the role vocabulary already lives — which ids this
   * machine has and what each one is — so the Bluetooth page picks from it rather than
   * carrying a hardcoded list that would drift from the firmware's.
   */
  const getCommsPeripheralEntries = useCallback((): PeripheralEntry[] => {
    if (!machineDefinition) {
      return [];
    }
    return Array.from(machineDefinition.peripherals.entries()).filter(
      ([, definition]) => definition.via_comms_mcu
    );
  }, [machineDefinition]);

  const value: MachineContextType = useMemo(() => ({
    machineDefinition,
    getBoilerName,
    getGroupName,
    getWaterTapName,
    getSteamWandName,
    getTankName,
    getBoilerCount,
    getGroupCount,
    getWaterTapCount,
    getSteamWandCount,
    hasBoilerSensor,
    hasGroupSensor,
    getBoilerControlModes,
    getGroupControlModes,
    getBoilerEntries,
    getGroupEntries,
    getWaterTapEntries,
    getSteamWandEntries,
    getCommsPeripheralEntries,
  }), [
    machineDefinition,
    getBoilerName,
    getGroupName,
    getWaterTapName,
    getSteamWandName,
    getTankName,
    getBoilerCount,
    getGroupCount,
    getWaterTapCount,
    getSteamWandCount,
    hasBoilerSensor,
    hasGroupSensor,
    getBoilerControlModes,
    getGroupControlModes,
    getBoilerEntries,
    getGroupEntries,
    getWaterTapEntries,
    getSteamWandEntries,
    getCommsPeripheralEntries,
  ]);

  return (
    <MachineContext.Provider value={value}>
      {children}
    </MachineContext.Provider>
  );
}

export function useMachine(): MachineContextType {
  const context = useContext(MachineContext);
  if (context === undefined) {
    throw new Error('useMachine must be used within a MachineProvider');
  }
  return context;
}
