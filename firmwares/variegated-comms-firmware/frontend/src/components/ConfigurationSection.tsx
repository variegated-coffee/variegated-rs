import { ComponentChildren } from 'preact';

interface ConfigurationSectionProps {
  title: string;
  children: ComponentChildren;
}

export const ConfigurationSection = ({ title, children }: ConfigurationSectionProps) => {
  return (
    <div
      style={{
        marginBottom: '1rem',
        backgroundColor: 'white',
        border: '1px solid #ddd',
        borderRadius: '8px',
        overflow: 'hidden'
      }}
    >
      <div
        style={{
          padding: '0.75rem 1rem',
          backgroundColor: '#f8f9fa',
          borderBottom: '1px solid #ddd',
          fontWeight: '600',
          fontSize: '0.95rem'
        }}
      >
        {title}
      </div>
      <div style={{ padding: '1rem' }}>
        {children}
      </div>
    </div>
  );
};
