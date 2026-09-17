package sim.runner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import sim.bridge.BridgeClient;
import sim.bridge.Json;
import sim.bridge.SensorFrame;
import sim.sdk.SimGamepad;
import sim.sdk.SimHardwareMap;
import sim.sdk.SimHub;
import sim.sdk.SimLocalizer;
import sim.sdk.SimBallCounter;
import sim.sdk.SimTargetProvider;
import sim.sdk.SimTelemetry;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;

/**
 * Runs a real FTC OpMode against the simulated world.
 *
 *   java -cp out;out-teamcode sim.runner.Main --opmode "TeleOp Main" --ws ws://localhost:8765
 *
 * The lifecycle is the hub's: construct, init, INIT loop until the world says START,
 * run, then STOP when the match clock says so.
 */
public class Main {

    public static void main(String[] args) throws Exception {
        String opModeName = arg(args, "--opmode", null);
        String ws = arg(args, "--ws", "ws://localhost:8765");
        String robotJson = arg(args, "--robot", "config/robot.json");
        String classes = arg(args, "--classes", "java/out-teamcode");
        boolean list = has(args, "--list");
        boolean echo = has(args, "--echo");

        List<OpModeRegistry.Entry> entries = OpModeRegistry.scan(new File(classes));
        if (list || opModeName == null) {
            System.out.println("OpModes found in " + classes + ":");
            for (OpModeRegistry.Entry e : entries) System.out.println("  " + e);
            if (opModeName == null) {
                System.out.println("\nPick one with --opmode \"<name>\"");
                return;
            }
        }

        OpModeRegistry.Entry chosen = null;
        for (OpModeRegistry.Entry e : entries) {
            if (e.name.equalsIgnoreCase(opModeName) || e.cls.getSimpleName().equalsIgnoreCase(opModeName)) {
                chosen = e;
                break;
            }
        }
        if (chosen == null) {
            System.err.println("No OpMode called \"" + opModeName + "\". Use --list.");
            System.exit(2);
            return;
        }

        SimHub hub = new SimHub();
        SimHardwareMap hw = buildHardwareMap(hub, robotJson);
        SimTelemetry telemetry = new SimTelemetry(hub);
        telemetry.setEcho(echo);
        Gamepad gamepad1 = new Gamepad();
        Gamepad gamepad2 = new Gamepad();

        BridgeClient bridge = new BridgeClient();
        System.out.println("connecting to " + ws + " ...");
        bridge.connect(ws);
        System.out.println("running " + chosen);

        Host host = new Host(hub, bridge, hw, telemetry, gamepad1, gamepad2);
        host.run(chosen);
        bridge.close();
        System.out.println("opmode finished");
    }

    /** Build the fake hardware from the same robot.json the world builds its body from. */
    private static SimHardwareMap buildHardwareMap(SimHub hub, String path) throws Exception {
        String text = new String(Files.readAllBytes(Paths.get(path)), StandardCharsets.UTF_8);
        Map<String, Object> robot = Json.parseObject(text);
        Map<String, Object> hardware = Json.obj(robot, "hardware");
        Map<String, Object> motorTable = Json.parseObject(new String(
                Files.readAllBytes(Paths.get(new File(path).getParent(), "motors.json")), StandardCharsets.UTF_8));
        Map<String, Object> variants = Json.obj(motorTable, "variants");

        SimHardwareMap hw = new SimHardwareMap(hub);

        // motors, by the role names the world uses on the wire
        Map<String, Object> drive = Json.obj(Json.obj(robot, "drivetrain"), "motors");
        for (String role : new String[] { "fl", "fr", "bl", "br" }) {
            hw.registerMotor(role, ticks(variants, Json.obj(drive, role)));
        }
        hw.registerMotor("intake", ticks(variants, Json.obj(Json.obj(robot, "intake"), "motor")));
        hw.registerMotor("transfer", ticks(variants, Json.obj(Json.obj(robot, "transfer"), "motor")));
        hw.registerMotor("flywheel", ticks(variants, Json.obj(Json.obj(robot, "flywheel"), "motor")));
        if (Json.bool(Json.obj(robot, "turret"), "enabled", false)) {
            hw.registerMotor("turret", ticks(variants, Json.obj(Json.obj(robot, "turret"), "motor")));
        }
        hw.registerServo("hood");
        hw.registerServo("gate");
        hw.registerImu("imu");
        hw.registerDistance("front");

        // The sim-only providers, looked up by name. On the hub these simply are not there.
        hw.register("localizer", new SimLocalizer(hub));
        hw.register("target", new SimTargetProvider(hub));
            hw.register("hopper", new SimBallCounter(hub));

        // TeamCode addresses devices by the names in robot.json -> hardware, so alias them.
        for (Map.Entry<String, Object> e : hardware.entrySet()) {
            String role = e.getKey();
            String configured = String.valueOf(e.getValue());
            if (hw.has(role) && !role.equals(configured)) hw.alias(role, configured);
        }
        return hw;
    }

    private static double ticks(Map<String, Object> variants, Map<String, Object> motorSpec) {
        String variant = Json.str(motorSpec, "variant", "5203-312");
        double gear = Json.num(motorSpec, "gearRatio", 1);
        return Json.num(Json.obj(variants, variant), "ticksPerRev", 537.7) * gear;
    }

    private static String arg(String[] args, String key, String dflt) {
        for (int i = 0; i < args.length - 1; i++) if (args[i].equals(key)) return args[i + 1];
        return dflt;
    }

    private static boolean has(String[] args, String key) {
        for (String a : args) if (a.equals(key)) return true;
        return false;
    }

    // ------------------------------------------------------------ host

    /**
     * Drives one OpMode through the hub's lifecycle. A LinearOpMode blocks, so it runs on
     * its own thread and this class feeds it frames; an OpMode is called directly.
     */
    static class Host implements LinearOpMode.LinearOpModeHost {
        private final SimHub hub;
        private final BridgeClient bridge;
        private final SimHardwareMap hw;
        private final SimTelemetry telemetry;
        private final Gamepad gamepad1, gamepad2;

        private volatile boolean started = false;
        private volatile boolean stopRequested = false;
        private volatile boolean frameReady = false;
        private final Object gate = new Object();

        Host(SimHub hub, BridgeClient bridge, SimHardwareMap hw, SimTelemetry telemetry, Gamepad g1, Gamepad g2) {
            this.hub = hub; this.bridge = bridge; this.hw = hw;
            this.telemetry = telemetry; this.gamepad1 = g1; this.gamepad2 = g2;
        }

        @Override public boolean isStarted() { return started; }
        @Override public boolean isStopRequested() { return stopRequested; }

        /** Block the OpMode thread until the world has produced another frame. */
        @Override public void idle() throws InterruptedException {
            synchronized (gate) {
                long waited = 0;
                while (!frameReady && !stopRequested && waited < 5000) {
                    gate.wait(50);
                    waited += 50;
                }
                frameReady = false;
            }
        }

        @Override public void sleep(long ms) throws InterruptedException {
            double until = hub.simTime() + ms / 1000.0;
            while (!stopRequested && hub.simTime() < until) idle();
        }

        void run(OpModeRegistry.Entry entry) throws Exception {
            Object instance = entry.cls.getDeclaredConstructor().newInstance();
            Thread opThread = null;

            if (instance instanceof LinearOpMode) {
                final LinearOpMode op = (LinearOpMode) instance;
                op.hardwareMap = hw;
                op.telemetry = telemetry;
                op.gamepad1 = gamepad1;
                op.gamepad2 = gamepad2;
                op.host = this;
                opThread = new Thread(() -> {
                    try {
                        op.runOpMode();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    } catch (Throwable t) {
                        System.err.println("OpMode threw:");
                        t.printStackTrace();
                    }
                    stopRequested = true;
                }, "opmode");
                opThread.setDaemon(true);
            }

            OpMode iterative = instance instanceof OpMode ? (OpMode) instance : null;
            if (iterative != null) {
                iterative.hardwareMap = hw;
                iterative.telemetry = telemetry;
                iterative.gamepad1 = gamepad1;
                iterative.gamepad2 = gamepad2;
                iterative.init();
            }

            boolean threadStarted = false;
            boolean iterativeStarted = false;
            ElapsedTime watchdog = new ElapsedTime();
            long seq = 0;

            while (!bridge.isClosed()) {
                SensorFrame frame = bridge.take(2000);
                if (frame == null) {
                    if (stopRequested) break;
                    continue;
                }
                hub.push(frame);
                hub.newLoop();
                com.qualcomm.robotcore.util.ElapsedTime.setSimTime(frame.t());
                SimGamepad.fill(gamepad1, frame.gamepad(1));
                SimGamepad.fill(gamepad2, frame.gamepad(2));

                if (!started && frame.started()) started = true;
                if (started && "FINISHED".equals(frame.period())) stopRequested = true;

                if (opThread != null && !threadStarted) {
                    threadStarted = true;
                    opThread.start();
                }

                hub.out.clear();

                if (iterative != null) {
                    if (!started) {
                        iterative.init_loop();
                    } else {
                        if (!iterativeStarted) { iterative.start(); iterativeStarted = true; }
                        if (!stopRequested) iterative.loop();
                    }
                } else {
                    // Let the LinearOpMode thread take exactly one pass over this frame.
                    watchdog.reset();
                    synchronized (gate) {
                        frameReady = true;
                        gate.notifyAll();
                    }
                    Thread.sleep(0, 200000);
                    if (watchdog.milliseconds() > 100) {
                        System.err.println("loop overrun: " + watchdog.milliseconds() + " ms");
                    }
                }

                bridge.send(hub.out.toJson(seq++));
                hw.frameSent(); // clear sticky per-frame flags now that the frame has gone

                if (stopRequested) {
                    if (iterative != null) iterative.stop();
                    break;
                }
            }

            stopRequested = true;
            synchronized (gate) { gate.notifyAll(); }
            if (opThread != null) opThread.join(1500);
        }
    }
}
