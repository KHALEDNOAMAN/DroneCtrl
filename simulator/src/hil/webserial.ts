import { Transport } from './HilSession';

/**
 * WebSerial transport for the HIL session: the ESP32's USB serial port,
 * straight from the browser. Chromium based browsers only (Chrome, Edge,
 * Opera), and only on a secure origin, which the Vercel deployment and
 * localhost both are.
 *
 * The esp32_hil firmware runs the port at 921600 baud. A 250 Hz stream of
 * HIL_SENSOR plus the replies is around 45 kB/s, which 115200 cannot carry.
 */

// The DOM lib does not type WebSerial yet. This is the subset used here.
interface SerialPortLike {
  open(options: { baudRate: number }): Promise<void>;
  close(): Promise<void>;
  readable: ReadableStream<Uint8Array> | null;
  writable: WritableStream<Uint8Array> | null;
}
interface SerialLike {
  requestPort(): Promise<SerialPortLike>;
}

export const HIL_BAUD = 921600;

export function webSerialAvailable(): boolean {
  return typeof navigator !== 'undefined' && 'serial' in navigator;
}

export interface SerialLink extends Transport {
  close(): Promise<void>;
}

/** Ask the user to pick the board's port, open it, and start reading. */
export async function openSerialLink(baudRate = HIL_BAUD): Promise<SerialLink> {
  const serial = (navigator as unknown as { serial: SerialLike }).serial;
  const port = await serial.requestPort();
  await port.open({ baudRate });

  const writer = port.writable!.getWriter();
  const reader = port.readable!.getReader();
  const sink: { onData: ((b: Uint8Array) => void) | null } = { onData: null };
  let closed = false;

  // Pump the port into the session until it is closed.
  (async () => {
    try {
      while (!closed) {
        const { value, done } = await reader.read();
        if (done) break;
        if (value && sink.onData) sink.onData(value);
      }
    } catch {
      // Unplugged or closed underneath us; the session sees timeouts.
    }
  })();

  return {
    write: (bytes) => writer.write(bytes),
    onData: (cb) => { sink.onData = cb; },
    close: async () => {
      closed = true;
      try { await reader.cancel(); } catch { /* already gone */ }
      reader.releaseLock();
      writer.releaseLock();
      await port.close();
    },
  };
}
