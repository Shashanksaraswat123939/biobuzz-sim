package sim.runner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The hub scans its TeamCode package for annotated classes and shows them in a list.
 * So does this, by walking the compiled output -- the OpMode you pick here is the same
 * one you would pick on a Driver Station.
 */
public class OpModeRegistry {
    public static class Entry {
        public final Class<?> cls;
        public final String name;
        public final String group;
        public final boolean auto;
        Entry(Class<?> cls, String name, String group, boolean auto) {
            this.cls = cls; this.name = name; this.group = group; this.auto = auto;
        }
        @Override public String toString() { return name + "  [" + (auto ? "auto" : "teleop") + ", " + group + "]"; }
    }

    private static final String PKG = "org.firstinspires.ftc.teamcode";

    public static List<Entry> scan(File classesRoot) {
        List<Entry> found = new ArrayList<Entry>();
        File pkgDir = new File(classesRoot, PKG.replace('.', File.separatorChar));
        collect(pkgDir, PKG, found);
        Collections.sort(found, (a, b) -> a.name.compareToIgnoreCase(b.name));
        return found;
    }

    private static void collect(File dir, String pkg, List<Entry> out) {
        File[] files = dir.listFiles();
        if (files == null) return;
        for (File f : files) {
            if (f.isDirectory()) {
                collect(f, pkg + "." + f.getName(), out);
            } else if (f.getName().endsWith(".class") && f.getName().indexOf('$') < 0) {
                String cn = pkg + "." + f.getName().substring(0, f.getName().length() - 6);
                try {
                    Class<?> c = Class.forName(cn);
                    if (c.isAnnotationPresent(Disabled.class)) continue;
                    Autonomous a = c.getAnnotation(Autonomous.class);
                    TeleOp t = c.getAnnotation(TeleOp.class);
                    if (a != null) {
                        out.add(new Entry(c, a.name().isEmpty() ? c.getSimpleName() : a.name(), a.group(), true));
                    } else if (t != null) {
                        out.add(new Entry(c, t.name().isEmpty() ? c.getSimpleName() : t.name(), t.group(), false));
                    }
                } catch (Throwable ignored) {
                    // a class that will not load is not an OpMode we can run
                }
            }
        }
    }
}
