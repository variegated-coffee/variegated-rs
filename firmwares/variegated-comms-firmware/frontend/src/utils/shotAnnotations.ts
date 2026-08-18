import { ShotAnnotation, ShotAnnotationKey, ShotAnnotations, ShotAnnotationValue } from '../schemas/schemas';

/**
 * Reading and editing a shot's annotation block.
 *
 * The block is a small ordered map: keys are unique, order is insertion order, and that
 * order is what a UI renders in. Everything here returns a new block rather than mutating
 * one, so Preact sees a changed reference.
 */

/**
 * The machine's `MAX_SHOT_ANNOTATIONS`.
 *
 * Duplicated from Rust because a bounded `heapless::Vec` reaches TypeScript as a plain
 * sequence with no length carried in the schema. It is not merely advisory: the firmware
 * decodes the body into that bounded vector, so a ninth entry fails to deserialize and
 * the PUT comes back 400. Keeping to it here turns that into a disabled button rather
 * than a rejected save.
 */
export const MAX_SHOT_ANNOTATIONS = 8;

/**
 * The keys with a dedicated variant, as a discriminant string.
 *
 * All of them are the *user's*. Machine-derived facts about a shot -- the routine it was
 * pulled with, when it started, how it ended -- are fields of `ShotLogMetadata` beside
 * this block, not entries in it. That separation is what makes replacing the whole block
 * safe: an edit can only overwrite what a user typed.
 */
export type NamedKey = 'DoseWeight' | 'Beans' | 'GrindSize';

function keysEqual(a: ShotAnnotationKey, b: ShotAnnotationKey): boolean {
  if (a.type !== b.type) return false;
  // `Other` is compared by content -- two `Other('water')` keys are the same key, which
  // is what makes the upsert below an upsert rather than an append.
  if (a.type === 'Other' && b.type === 'Other') return a.value === b.value;
  return true;
}

export function findAnnotation(
  annotations: ShotAnnotations,
  key: ShotAnnotationKey
): ShotAnnotation | undefined {
  return annotations.entries.find((entry) => keysEqual(entry.key, key));
}

/** The value for a named key, or `undefined`. */
export function namedValue(
  annotations: ShotAnnotations,
  key: NamedKey
): ShotAnnotationValue | undefined {
  return findAnnotation(annotations, { type: key } as ShotAnnotationKey)?.value;
}

/** A value rendered for display. */
export function formatValue(value: ShotAnnotationValue): string {
  switch (value.type) {
    case 'Number':
      // Trailing zeros trimmed: a dose reads "18.3" or "18", not "18.300000".
      return String(Number(value.value.toFixed(2)));
    case 'Text':
      return value.value;
  }
}

/** A key rendered for display. */
export function formatKey(key: ShotAnnotationKey): string {
  switch (key.type) {
    case 'DoseWeight':
      return 'Dose';
    case 'Beans':
      return 'Beans';
    case 'GrindSize':
      return 'Grind';
    case 'Other':
      return key.value;
  }
}

/**
 * Insert or replace the value for `key`.
 *
 * Returns `null` when the block is full and the key is new -- the same refusal the
 * machine makes, surfaced early so a user is told before they lose what they typed.
 *
 * Spreads the block rather than rebuilding it as `{ entries }`, and that is load-bearing
 * rather than style: `entries` is no longer the only field. From format version 5 the block
 * also carries `tasting_notes`, which this file never touches -- and rebuilding from
 * `entries` alone would drop it on every save, erasing a note nobody asked to change. The
 * spread also means the next field added needs no edit here.
 */
export function upsert(
  annotations: ShotAnnotations,
  key: ShotAnnotationKey,
  value: ShotAnnotationValue
): ShotAnnotations | null {
  const index = annotations.entries.findIndex((entry) => keysEqual(entry.key, key));
  if (index >= 0) {
    const entries = [...annotations.entries];
    entries[index] = { key, value };
    return { ...annotations, entries };
  }
  if (annotations.entries.length >= MAX_SHOT_ANNOTATIONS) {
    return null;
  }
  return { ...annotations, entries: [...annotations.entries, { key, value }] };
}

/** Drop a key, preserving the order of what is left. */
export function remove(annotations: ShotAnnotations, key: ShotAnnotationKey): ShotAnnotations {
  return {
    ...annotations,
    entries: annotations.entries.filter((entry) => !keysEqual(entry.key, key)),
  };
}

/**
 * Set a named key from a text field, removing it when the field is blank.
 *
 * Blank-means-remove is what makes an edit form round-trip: a user clearing the beans
 * field expects the beans annotation to go away, not to become an empty string that
 * renders as a chip with no content.
 */
export function setText(
  annotations: ShotAnnotations,
  key: NamedKey,
  text: string
): ShotAnnotations | null {
  const trimmed = text.trim();
  const asKey = { type: key } as ShotAnnotationKey;
  if (trimmed === '') {
    return remove(annotations, asKey);
  }
  return upsert(annotations, asKey, { type: 'Text', value: trimmed });
}

/** The text of a named key, or `''` if unset or not textual. */
export function textOf(annotations: ShotAnnotations, key: NamedKey): string {
  const value = namedValue(annotations, key);
  return value?.type === 'Text' ? value.value : '';
}

/** The dose in grams, if one was recorded as a number. */
export function doseWeight(annotations: ShotAnnotations): number | null {
  const value = namedValue(annotations, 'DoseWeight');
  return value?.type === 'Number' ? value.value : null;
}
