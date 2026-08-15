// Global type definitions

interface PostcardLibrary {
  serialize<T>(data: T): Promise<Uint8Array>;
  deserialize<T>(data: Uint8Array): Promise<T>;
}

interface Window {
  postcard: PostcardLibrary;
}
