#!/usr/bin/env tsx
/**
 * Check the generated schemas against values Rust actually serialized.
 *
 * Two independent comparisons per fixture, because neither subsumes the other:
 *
 *   1. Decode the postcard bytes, re-encode, assert byte-identical. This catches wrong
 *      node kinds, wrong nesting, wrong field counts and wrong variant order. It is
 *      completely blind to field *names* and to two same-typed fields being swapped,
 *      because postcard encodes neither.
 *
 *   2. Normalize the decoded value into serde_json's shape and deep-compare against the
 *      JSON Rust wrote. This is the half that catches names and transpositions -- the
 *      exact bug class the hand-maintained schema file suffered from.
 *
 * Shipping only (1) would let `struct { pressure: f32, temperature: f32 }` be described
 * with its fields swapped and still pass forever.
 */

import { readFileSync } from 'fs';
import { dirname, join } from 'path';
import { fileURLToPath } from 'url';
import { deserialize, serialize } from '@variegated-coffee/serde-postcard-ts';
import * as schemas from '../src/schemas/schemas';

const here = dirname(fileURLToPath(import.meta.url));
const fixturesDir = join(here, '..', 'fixtures');

interface FixtureEntry {
  name: string;
  /** Export name in schemas.ts, or `seq:<name>` for a sequence of that type. */
  schema: string;
}

/** Anything the decoder can hand back. */
type Decoded = unknown;

/** The runtime schema descriptors are structurally typed; this is the shape we walk. */
interface SchemaNode {
  kind: string;
  inner?: SchemaNode;
  item?: SchemaNode;
  items?: readonly SchemaNode[];
  key?: SchemaNode;
  value?: SchemaNode;
  fields?: Record<string, SchemaNode>;
  variants?: Record<string, SchemaNode>;
  name?: string;
}

function resolveSchema(spec: string): SchemaNode {
  const seq = spec.startsWith('seq:');
  const exportName = seq ? spec.slice(4) : spec;
  const found = (schemas as Record<string, unknown>)[exportName];
  if (found === undefined) {
    throw new Error(`schemas.ts does not export ${exportName}`);
  }
  const node = found as SchemaNode;
  return seq ? ({ kind: 'seq', item: node } as SchemaNode) : node;
}

/**
 * Rewrite a decoded value into the shape serde_json would have produced.
 *
 * The two disagree in exactly three places, and the schema tree is what tells us which
 * one applies at each position:
 *   - enums: serde_json writes `"Unit"` or `{Variant: payload}`; the decoder produces
 *     `{type, value}`.
 *   - maps: serde_json writes an object with stringified keys; the decoder produces a Map.
 *   - u64/i64: serde_json writes a number; the decoder produces a bigint.
 */
function normalize(value: Decoded, schema: SchemaNode): Decoded {
  switch (schema.kind) {
    case 'option':
      // serde_json writes `null` for None, and the payload bare for Some.
      return value === null || value === undefined
        ? null
        : normalize(value, schema.inner as SchemaNode);

    case 'seq':
      return (value as Decoded[]).map((v) => normalize(v, schema.item as SchemaNode));

    case 'tuple':
    case 'tuple_struct':
    case 'tuple_variant':
      return (value as Decoded[]).map((v, i) =>
        normalize(v, (schema.items as readonly SchemaNode[])[i])
      );

    case 'map': {
      const out: Record<string, Decoded> = {};
      for (const [k, v] of value as Map<unknown, Decoded>) {
        out[String(k)] = normalize(v, schema.value as SchemaNode);
      }
      return out;
    }

    case 'struct':
    case 'struct_variant': {
      const out: Record<string, Decoded> = {};
      const fields = schema.fields as Record<string, SchemaNode>;
      for (const [name, fieldSchema] of Object.entries(fields)) {
        out[name] = normalize((value as Record<string, Decoded>)[name], fieldSchema);
      }
      return out;
    }

    case 'enum': {
      const tagged = value as { type: string; value?: Decoded };
      const variant = (schema.variants as Record<string, SchemaNode>)[tagged.type];
      if (variant === undefined) {
        throw new Error(`decoded unknown variant ${tagged.type}`);
      }
      // A unit variant is a bare string in serde_json; everything else is a
      // single-key object.
      if (variant.kind === 'unit_variant') return tagged.type;
      return { [tagged.type]: normalize(tagged.value, variant) };
    }

    case 'newtype_struct':
    case 'newtype_variant':
      return normalize(value, schema.inner as SchemaNode);

    case 'u64':
    case 'i64':
    case 'u128':
    case 'i128':
      // Fixtures keep these below 2^53 precisely so this is lossless.
      return Number(value as bigint);

    default:
      return value;
  }
}

function deepEqual(a: Decoded, b: Decoded, path: string, problems: string[]): void {
  if (typeof a === 'number' && typeof b === 'number') {
    // f32 decoded into a double on one side and formatted by ryu on the other.
    // Comparing at f32 precision is the only meaningful test.
    if (Math.fround(a) !== Math.fround(b) && !(Number.isNaN(a) && Number.isNaN(b))) {
      problems.push(`${path}: ${a} !== ${b}`);
    }
    return;
  }

  if (a === null || b === null || typeof a !== 'object' || typeof b !== 'object') {
    if (a !== b) problems.push(`${path}: ${JSON.stringify(a)} !== ${JSON.stringify(b)}`);
    return;
  }

  if (Array.isArray(a) !== Array.isArray(b)) {
    problems.push(`${path}: array/object mismatch`);
    return;
  }

  if (Array.isArray(a) && Array.isArray(b)) {
    if (a.length !== b.length) {
      problems.push(`${path}: length ${a.length} !== ${b.length}`);
      return;
    }
    a.forEach((v, i) => deepEqual(v, b[i], `${path}[${i}]`, problems));
    return;
  }

  const ao = a as Record<string, Decoded>;
  const bo = b as Record<string, Decoded>;
  const keys = new Set([...Object.keys(ao), ...Object.keys(bo)]);
  for (const key of keys) {
    if (!(key in ao)) {
      problems.push(`${path}.${key}: missing from decoded value`);
      continue;
    }
    if (!(key in bo)) {
      problems.push(`${path}.${key}: missing from Rust's JSON`);
      continue;
    }
    deepEqual(ao[key], bo[key], `${path}.${key}`, problems);
  }
}

function hex(bytes: Uint8Array, limit = 24): string {
  const shown = Array.from(bytes.slice(0, limit))
    .map((b) => b.toString(16).padStart(2, '0'))
    .join(' ');
  return bytes.length > limit ? `${shown} ...` : shown;
}

/** Index of the first differing byte, or -1. */
function firstDifference(a: Uint8Array, b: Uint8Array): number {
  const n = Math.min(a.length, b.length);
  for (let i = 0; i < n; i++) if (a[i] !== b[i]) return i;
  return a.length === b.length ? -1 : n;
}

function main() {
  const index = JSON.parse(
    readFileSync(join(fixturesDir, 'index.json'), 'utf8')
  ) as FixtureEntry[];

  let failed = 0;

  for (const entry of index) {
    const schema = resolveSchema(entry.schema);
    const bytes = new Uint8Array(readFileSync(join(fixturesDir, `${entry.name}.bin`)));
    const expectedJson = JSON.parse(
      readFileSync(join(fixturesDir, `${entry.name}.json`), 'utf8')
    ) as Decoded;

    const problems: string[] = [];

    // 1. decode
    let decoded: Decoded;
    try {
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      decoded = deserialize(schema as any, bytes).value;
    } catch (e) {
      console.error(`✗ ${entry.name}: decode threw: ${String(e)}`);
      console.error(`    bytes: ${hex(bytes)}`);
      failed++;
      continue;
    }

    // 2. re-encode, byte-identical
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const reencoded = serialize(schema as any, decoded);
    const diff = firstDifference(bytes, reencoded);
    if (diff !== -1) {
      problems.push(
        `re-encoded bytes differ at offset ${diff} ` +
          `(${bytes.length} vs ${reencoded.length} bytes)\n` +
          `      rust: ${hex(bytes)}\n` +
          `        ts: ${hex(reencoded)}`
      );
    }

    // 3. structural comparison against Rust's own JSON
    try {
      deepEqual(normalize(decoded, schema), expectedJson, entry.name, problems);
    } catch (e) {
      problems.push(`normalization failed: ${String(e)}`);
    }

    if (problems.length > 0) {
      console.error(`✗ ${entry.name} (${entry.schema})`);
      for (const p of problems) console.error(`    ${p}`);
      failed++;
    } else {
      console.log(
        `✓ ${entry.name.padEnd(20)} ${String(bytes.length).padStart(5)} B  ` +
          `decode ok, re-encode byte-identical, matches Rust's JSON`
      );
    }
  }

  if (failed > 0) {
    console.error(`\n${failed} of ${index.length} fixtures failed`);
    process.exit(1);
  }
  console.log(`\nall ${index.length} fixtures pass`);
}

main();
