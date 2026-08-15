#!/usr/bin/env tsx
/**
 * Assert that every schema in the generated module is defined at load time.
 *
 * The generated file is a list of `const`s evaluated top to bottom, so a schema that
 * references another one declared further down does not merely look untidy -- it is a
 * temporal-dead-zone `ReferenceError` thrown while the module initializes. In a bundled
 * SPA that surfaces as a blank page with a stack pointing into vendor code, so it is
 * worth failing here instead.
 *
 * Merely importing the module is most of the test: if the ordering were wrong, this
 * script would throw before reaching main(). The explicit checks then catch the subtler
 * case where a value is `undefined` rather than absent.
 */

import * as schemas from '../src/schemas/schemas';

function main() {
  const entries = Object.entries(schemas);
  const undefinedExports = entries.filter(([, value]) => value === undefined);

  if (undefinedExports.length > 0) {
    console.error('These exports are undefined at module load:');
    for (const [name] of undefinedExports) console.error(`  ${name}`);
    process.exit(1);
  }

  const schemaExports = entries.filter(([name]) => name.endsWith('Schema'));
  const missingKind = schemaExports.filter(
    ([, value]) => typeof (value as { kind?: unknown }).kind !== 'string'
  );

  if (missingKind.length > 0) {
    console.error('These exports are not schema descriptors:');
    for (const [name] of missingKind) console.error(`  ${name}`);
    process.exit(1);
  }

  console.log(
    `schema module loads cleanly: ${schemaExports.length} schemas, all initialized`
  );
}

main();
