import { ComponentChildren } from 'preact';
import { Button, tokens } from '@variegated-coffee/ui';

interface ConfigurationSectionProps {
  title: string;
  children: ComponentChildren;
}

/**
 * A titled card that does not collapse.
 *
 * Deliberately not `Section` from the design system, which is the collapsible one. These
 * hold a handful of fields each and are already inside a collapsible parent; making them
 * collapse too would give the configuration screen two nested levels of disclosure and a
 * reader no way to tell which one hid what they were looking for.
 */
export const ConfigurationSection = ({ title, children }: ConfigurationSectionProps) => {
  return (
    <div
      style={{
        marginBottom: tokens.space.md,
        backgroundColor: tokens.color.surfaceRaised,
        border: `1px solid ${tokens.color.border}`,
        borderRadius: tokens.radius.md,
        overflow: 'hidden',
      }}
    >
      <div
        style={{
          padding: `${tokens.space.sm} ${tokens.space.md}`,
          backgroundColor: tokens.color.surfaceSunken,
          borderBottom: `1px solid ${tokens.color.border}`,
          fontWeight: 600,
          fontSize: '0.95rem',
        }}
      >
        {title}
      </div>
      <div style={{ padding: tokens.space.md }}>{children}</div>
    </div>
  );
};

/**
 * The name of the entity a detail screen is about, and its index.
 *
 * The five detail components each rebuilt this, and each phrased the index differently --
 * "Tank ID: 0", "Steam Wand ID: 0", "Boiler Index: 0". The index is worth showing (it is
 * how the entity is addressed on the wire and in a schedule command) but it is metadata
 * about the heading rather than part of it, so it is muted and monospace, and phrased the
 * same way everywhere.
 */
export const EntityDetailHeader = ({ name, index }: { name: string; index: number }) => (
  <div style={{ marginBottom: tokens.space.lg }}>
    <h2 style={{ margin: 0, fontSize: '1.3rem' }}>{name}</h2>
    <div
      style={{
        fontSize: '0.85rem',
        color: tokens.color.inkMuted,
        marginTop: tokens.space.xs,
        fontFamily: tokens.font.mono,
      }}
    >
      index {index}
    </div>
  </div>
);

/**
 * An optional measurement, and the em dash that stands in when there is none.
 *
 * "Not set", "N/A" and "Not configured" were three spellings of one state across these
 * screens. A dash says the same thing without implying which of the three it is.
 */
export const optionalValue = (
  value: number | null | undefined,
  decimals: number
): string => (value !== null && value !== undefined ? value.toFixed(decimals) : '—');

/** The unit that goes with `optionalValue`, omitted when there is no value to carry one. */
export const optionalUnit = (
  value: number | null | undefined,
  unit: string
): string | undefined => (value !== null && value !== undefined ? unit : undefined);

/**
 * A setting that opens its own editor: a name, what it covers, and the way in.
 *
 * All five detail screens have several of these and each rebuilt the row. Two things they
 * disagreed about, which is why it is worth centralising rather than just tokenising:
 *
 * - **The button was blue when a setting existed and green when it did not** -- two filled
 *   colours for one action, differing only by a state the adjacent text already states. It
 *   is one secondary button whose *label* carries the difference: Edit or Configure.
 * - **The accessible name.** Ten rows reading just "Edit" give a screen-reader user ten
 *   identical buttons. The setting's name goes into `ariaLabel`.
 */
export const SettingRow = ({
  title,
  description,
  configured = true,
  onEdit,
}: {
  title: string;
  description: string;
  /** False when the machine holds nothing for this setting yet. */
  configured?: boolean;
  onEdit: () => void;
}) => (
  <div
    style={{
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
      gap: tokens.space.sm,
      flexWrap: 'wrap',
    }}
  >
    <div>
      <div style={{ fontSize: '0.9rem', color: configured ? tokens.color.ink : tokens.color.inkMuted }}>
        {configured ? title : `${title} — not configured`}
      </div>
      <div style={{ fontSize: '0.75rem', color: tokens.color.inkMuted, marginTop: '0.1rem' }}>
        {description}
      </div>
    </div>
    <Button
      variant="secondary"
      size="sm"
      onClick={onEdit}
      ariaLabel={`${configured ? 'Edit' : 'Configure'} ${title}`}
    >
      {configured ? 'Edit' : 'Configure'}
    </Button>
  </div>
);
