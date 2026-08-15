import { memo } from 'preact/compat';

interface JsonModalProps {
  title: string;
  data: Record<string, unknown> | Map<unknown, unknown> | unknown[] | null;
  isOpen: boolean;
  onClose: () => void;
}

const JsonModalComponent = ({ title, data, isOpen, onClose }: JsonModalProps) => {
  if (!isOpen) return null;

  // Custom replacer to handle BigInt values and Maps
  const bigIntReplacer = (_key: string, value: unknown): unknown => {
    // Convert BigInt to string
    if (typeof value === 'bigint') {
      return value.toString() + 'n';
    }
    // Convert Map to object
    if (value instanceof Map) {
      const obj: Record<string, unknown> = {};
      value.forEach((val, key: unknown) => {
        const keyStr = typeof key === 'string' || typeof key === 'number' || typeof key === 'bigint' || typeof key === 'boolean'
          ? String(key)
          : JSON.stringify(key);
        obj[keyStr] = val;
      });
      return obj;
    }
    return value;
  };

  return (
    <div
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        backgroundColor: 'rgba(0, 0, 0, 0.5)',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        zIndex: 1000,
        padding: '2rem'
      }}
      onClick={onClose}
    >
      <div
        style={{
          backgroundColor: 'white',
          borderRadius: '8px',
          maxWidth: '900px',
          maxHeight: '80vh',
          width: '100%',
          display: 'flex',
          flexDirection: 'column',
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)'
        }}
        onClick={(e) => e.stopPropagation()}
      >
        {/* Header */}
        <div style={{
          padding: '1.5rem',
          borderBottom: '1px solid #ddd',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center'
        }}>
          <h2 style={{ margin: 0, fontSize: '1.5rem' }}>{title}</h2>
          <button
            onClick={onClose}
            style={{
              background: 'none',
              border: 'none',
              fontSize: '1.5rem',
              cursor: 'pointer',
              padding: '0.5rem',
              color: '#666',
              lineHeight: 1
            }}
            title="Close"
          >
            ×
          </button>
        </div>

        {/* Content */}
        <div style={{
          padding: '1.5rem',
          overflow: 'auto',
          flex: 1
        }}>
          <pre style={{
            fontSize: '0.875rem',
            background: '#f5f5f5',
            padding: '1rem',
            borderRadius: '4px',
            margin: 0,
            overflow: 'auto'
          }}>
            {JSON.stringify(data, bigIntReplacer, 2)}
          </pre>
        </div>
      </div>
    </div>
  );
};

export const JsonModal = memo(JsonModalComponent);
