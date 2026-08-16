import { useEffect } from 'preact/hooks';
import { ShotLogEvent } from '../schemas/schemas';

/**
 * Shot-log pushes, delivered to whoever is showing the list.
 *
 * A module-level store rather than state in `app.tsx`, mirroring `state/routineBodies.ts`
 * next to it. The alternative is threading a *stream* through a prop tree, which needs a
 * sequence number to tell two identical events apart and an effect to consume it -- and
 * gets the ordering wrong the first time a store and a delete arrive together.
 *
 * Nothing is retained. An event that arrives while the panel is unmounted is dropped, and
 * that is correct: the panel fetches a fresh page when it mounts.
 */

type Listener = (event: ShotLogEvent) => void;

const listeners = new Set<Listener>();

/** Called by the WebSocket service. */
export function publishShotLogEvent(event: ShotLogEvent): void {
  for (const listener of listeners) listener(event);
}

/** Returns an unsubscribe function. */
export function subscribeShotLogEvents(listener: Listener): () => void {
  listeners.add(listener);
  return () => {
    listeners.delete(listener);
  };
}

/**
 * Subscribe for the life of a component.
 *
 * Re-subscribes whenever `handler` changes, and the cleanup runs on every re-subscribe --
 * so a caller passing an inline function re-registers each render rather than leaking one
 * listener per render. Wrap the handler in `useCallback` to avoid the churn.
 */
export function useShotLogEvents(handler: Listener): void {
  useEffect(() => subscribeShotLogEvents(handler), [handler]);
}
