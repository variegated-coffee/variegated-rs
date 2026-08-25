import { memo } from 'preact/compat';
import { Button, Dialog, tokens } from '@variegated-coffee/ui';

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
    // This was the design system's only shared modal shell -- and every other modal in the
    // app rebuilt its backdrop rather than reusing it, because it is not a shell, it is a
    // JSON viewer. Now it is one use of the real shell, and gets the focus trap, the
    // Escape handler and the dialog semantics none of the hand-built ones had.
    <Dialog
      title={title}
      onClose={onClose}
      width="900px"
      footer={
        <Button variant="secondary" onClick={onClose}>
          Close
        </Button>
      }
    >
      <pre
        style={{
          font: `0.875rem ${tokens.font.mono}`,
          background: tokens.color.surfaceSunken,
          border: `1px solid ${tokens.color.border}`,
          padding: tokens.space.md,
          borderRadius: tokens.radius.sm,
          margin: 0,
          overflow: 'auto',
        }}
      >
        {JSON.stringify(data, bigIntReplacer, 2)}
      </pre>
    </Dialog>
  );
};

export const JsonModal = memo(JsonModalComponent);
