/**
 * The world's end of the bridge. The browser connects to the relay as a client, sends one
 * SensorFrame per world frame and applies the ActuatorFrame that comes back.
 *
 * Lockstep is deliberately NOT the default here: a dropped brain must not freeze the view.
 * In lockstep the world waits for the brain, which is what the deterministic tests want.
 */
import type { ActuatorFrame, SensorFrame } from '@core/types.js';

export type BridgeStatus = 'off' | 'connecting' | 'waiting' | 'live' | 'error';

export class WorldBridge {
  status: BridgeStatus = 'off';
  lastAct: ActuatorFrame = { seq: 0, motors: {}, servos: {} };
  brainConnected = false;
  private ws: WebSocket | null = null;
  private pending = false;

  constructor(private readonly url = 'ws://localhost:8765') {}

  connect(onChange: () => void): void {
    this.status = 'connecting';
    onChange();
    try {
      this.ws = new WebSocket(this.url);
    } catch {
      this.status = 'error';
      onChange();
      return;
    }
    this.ws.onopen = () => {
      this.ws?.send(JSON.stringify({ type: 'hello', role: 'world' }));
      this.status = 'waiting';
      onChange();
    };
    this.ws.onmessage = (e) => {
      let msg: Record<string, unknown>;
      try {
        msg = JSON.parse(String(e.data));
      } catch {
        return;
      }
      if (msg.type === 'actuator') {
        this.lastAct = msg as unknown as ActuatorFrame;
        this.pending = false;
        this.status = 'live';
        this.brainConnected = true;
        onChange();
      } else if (msg.type === 'brain') {
        this.brainConnected = Boolean(msg.connected);
        this.status = this.brainConnected ? 'live' : 'waiting';
        onChange();
      } else if (msg.type === 'reset') {
        this.lastAct = { seq: 0, motors: {}, servos: {} };
        this.brainConnected = false;
        this.status = 'waiting';
        onChange();
      }
    };
    this.ws.onclose = () => {
      this.status = 'off';
      this.brainConnected = false;
      this.ws = null;
      onChange();
    };
    this.ws.onerror = () => {
      this.status = 'error';
      onChange();
    };
  }

  disconnect(): void {
    this.ws?.close();
    this.ws = null;
    this.status = 'off';
    this.brainConnected = false;
  }

  get connected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN;
  }

  /** Send this frame's sensors. Returns the actuator frame to apply (last known). */
  exchange(sensors: SensorFrame): ActuatorFrame {
    if (this.connected && !this.pending) {
      this.ws!.send(JSON.stringify(sensors));
      this.pending = true;
      // One frame of latency is intended and matches the plan's lockstep timing note.
      setTimeout(() => (this.pending = false), 0);
    }
    return this.lastAct;
  }
}
