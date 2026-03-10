package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * Small helper to publish telemetry to the Elastic dashboard.
 *
 * Primitives are written to NetworkTables under the table "Elastic/<key>" so
 * Elastic (the WPILib dashboard) can read them via NT4/NetworkTables.
 *
 * For complex Sendable objects we delegate to SmartDashboard.putData so the
 * SendableRegistry/Sendable support continues to work (camera streams, Field2d,
 * SendableChooser, etc.).
 */
public final class ElasticDashboard {
    private static NetworkTableEntry entry(String key) {
        return NetworkTableInstance.getDefault().getEntry("Elastic/" + key);
    }

    public static void putNumber(String key, double value) {
        try {
            entry(key).setDouble(value);
        } catch (Exception ignored) {
        }
    }

    public static void putBoolean(String key, boolean value) {
        try {
            entry(key).setBoolean(value);
        } catch (Exception ignored) {
        }
    }

    public static void putString(String key, String value) {
        try {
            entry(key).setString(value == null ? "" : value);
        } catch (Exception ignored) {
        }
    }

    /**
     * Put a Sendable sized widget (delegates to SmartDashboard so WPILib Sendable
     * registration still works).
     */
    public static void putData(String key, Sendable data) {
        SmartDashboard.putData(key, data);
    }

    public static void putData(String key, SendableChooser<?> chooser) {
        SmartDashboard.putData(key, chooser);
    }

    public static void putData(String key, Field2d field) {
        SmartDashboard.putData(key, field);
    }
}
