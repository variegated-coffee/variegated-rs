import { ComponentChildren } from 'preact';
import { tokens } from '@variegated-coffee/ui';

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
