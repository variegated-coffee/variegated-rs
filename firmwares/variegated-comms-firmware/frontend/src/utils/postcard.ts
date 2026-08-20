import { serialize, deserialize, InferType, Schema } from '@variegated-coffee/serde-postcard-ts';

export async function fetchPostcard<S extends Schema>(url: string, schema: S): Promise<InferType<S>> {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`HTTP error! status: ${response.status}`);
  }
  const arrayBuffer = await response.arrayBuffer();
  const uint8Array = new Uint8Array(arrayBuffer);
  const result = deserialize(schema, uint8Array);
  return result.value;
}

export async function postPostcard<DS extends Schema, RS extends Schema | undefined = undefined>(
  url: string,
  data: InferType<DS>,
  dataSchema: DS,
  responseSchema?: RS
): Promise<RS extends Schema ? InferType<RS> : void> {
  const binary = serialize(dataSchema, data);
  const response = await fetch(url, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/octet-stream',
    },
    body: binary as BodyInit,
  });

  if (!response.ok) {
    throw new Error(`HTTP error! status: ${response.status}`);
  }

  if (responseSchema) {
    const arrayBuffer = await response.arrayBuffer();
    const uint8Array = new Uint8Array(arrayBuffer);
    const result = deserialize(responseSchema, uint8Array);
    return result.value as RS extends Schema ? InferType<RS> : void;
  }
  return undefined as RS extends Schema ? InferType<RS> : void;
}

export async function putPostcard<S extends Schema>(url: string, data: InferType<S>, dataSchema: S): Promise<void> {
  const binary = serialize(dataSchema, data);
  const response = await fetch(url, {
    method: 'PUT',
    headers: {
      'Content-Type': 'application/octet-stream',
    },
    body: binary as BodyInit,
  });

  if (!response.ok) {
    throw new Error(`HTTP error! status: ${response.status}`);
  }
}

// `postEmpty` was here — a bodyless POST for commands whose only argument was in the path.
// Every route that took that shape was a `/command/...` one, and they have all moved onto the
// WebSocket, so it lost its last caller with `tagDoseFromScale`.

export async function deleteRequest(url: string): Promise<void> {
  const response = await fetch(url, {
    method: 'DELETE',
  });

  if (!response.ok) {
    throw new Error(`HTTP error! status: ${response.status}`);
  }
}
