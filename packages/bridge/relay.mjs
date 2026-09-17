/**
 * The bridge, in the middle. The browser cannot host a WebSocket server, so the world
 * connects here as a client too and this just forwards:
 *
 *   world (browser)  --SensorFrame-->  relay  --SensorFrame-->  brain (Java)
 *   world (browser)  <--ActuatorFrame--  relay  <--ActuatorFrame--  brain (Java)
 *
 *   npm run relay
 */
import { WebSocketServer } from 'ws';

const PORT = Number(process.env.BRIDGE_PORT || 8765);
const wss = new WebSocketServer({ port: PORT });

let world = null;
let brain = null;
let frames = 0;
let commands = 0;

const label = (ws) => (ws === world ? 'world' : ws === brain ? 'brain' : 'unknown');

wss.on('connection', (ws) => {
  ws.on('message', (data) => {
    const text = data.toString();
    let msg;
    try {
      msg = JSON.parse(text);
    } catch {
      return;
    }

    if (msg.type === 'hello') {
      if (msg.role === 'brain') {
        if (brain) brain.close();
        brain = ws;
        console.log('brain connected');
        world?.send(JSON.stringify({ type: 'brain', connected: true }));
      } else {
        if (world) world.close();
        world = ws;
        console.log('world connected');
        brain?.send(JSON.stringify({ type: 'reset' }));
      }
      return;
    }

    if (msg.type === 'sensor') {
      frames++;
      if (brain && brain.readyState === 1) brain.send(text);
    } else if (msg.type === 'actuator') {
      commands++;
      if (world && world.readyState === 1) world.send(text);
    }
  });

  ws.on('close', () => {
    const who = label(ws);
    if (ws === world) {
      world = null;
      brain?.send(JSON.stringify({ type: 'reset' }));
    }
    if (ws === brain) {
      brain = null;
      world?.send(JSON.stringify({ type: 'brain', connected: false }));
    }
    if (who !== 'unknown') console.log(`${who} disconnected`);
  });
});

setInterval(() => {
  if (frames || commands) {
    console.log(`  ${frames} frames out, ${commands} commands back${brain ? '' : '  (no brain)'}`);
    frames = 0;
    commands = 0;
  }
}, 5000).unref?.();

console.log(`bridge relay on ws://localhost:${PORT}`);
console.log('  world: open the sim and switch Brain to "Java"');
console.log('  brain: java -cp "java/out;java/out-teamcode" sim.runner.Main --opmode "TeleOp Main"');
