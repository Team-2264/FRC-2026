package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;
import frc.robot.ElasticDashboard;

/**
 * Simple subsystem that exposes two independent NEO motors used for
 * quick manual actions. Motor IDs are configurable at construction.
 */
public class ManualMotors extends SubsystemBase {
    private final Neo rotator; // motor that will be commanded to rotate to a position
    private final Neo spinner; // motor that will spin while button held

    public ManualMotors(int rotatorCanId, int spinnerCanId) {
        NeoConfiguration rotCfg = new NeoConfiguration(rotatorCanId).withBrakeMode(true).withKP(0.5)
                .withKI(0.0).withKD(0.0);
        NeoConfiguration spinCfg = new NeoConfiguration(spinnerCanId).withBrakeMode(false).withKP(0.0)
                .withKI(0.0).withKD(0.0);

        rotator = new Neo(rotCfg);
        spinner = new Neo(spinCfg);
    }

    /**
     * Rotate rotator motor to absolute rotations (motor rotations).
     */
    public void rotateToPositionRevs(double revs) {
        rotator.rotateTo(revs);
    }

    /**
     * Convenience: rotate by a delta in rotations (relative to current encoder position).
     */
    public void rotateByDegrees(double degrees) {
        double deltaRevs = degrees / 360.0;
        double current = rotator.getPosition();
        rotator.rotateTo(current + deltaRevs);
    }

    /**
     * Returns current position in motor rotations.
     */
    public double getRotatorPositionRevs() {
        return rotator.getPosition();
    }

    /**
     * Start spinner at a duty cycle [-1..1].
     */
    public void startSpinner(double duty) {
        spinner.rotateAtSpeed(duty);
    }

    /**
     * Stop the spinner motor immediately.
     */
    public void stopSpinner() {
        spinner.stop();
    }

    @Override
    public void periodic() {
        // expose minimal telemetry for debugging
        ElasticDashboard.putNumber("Manual/RotatorPosRevs", getRotatorPositionRevs());
    }
}
