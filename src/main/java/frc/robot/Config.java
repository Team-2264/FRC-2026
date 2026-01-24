package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.vision.VisionContainer.VisionSelector;

/**
 * This defines the user settable configuration
 * <p>
 * Add settings to the appropriate interface to limit visibility of those parameters
 * <p>
 * If a parameter is shared by more than one interface, then define a parameter in common in the
 * high level Config interface and parameters that reference that common parameter to all the
 * appropriate interfaces. (That does reveal that parameter, though, in {@link Config})
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * <p>
 * There are a few user settable constants scattered around this project especially in
 * {@link AcquireAprilTag}. These likely will not be changed and aren't in {@link Config} or
 * {@link Constants}.
 */
public interface Config {
    // Variables for common use or used more than once can be put here and referenced in more than
    // one interface. Everything here is also visible by reference to the Config interface.

    // Select which Vision System should be activated
    public static VisionSelector visionSelector = VisionSelector.usePhotonVision;


    /**
     * import static PhotonVisionSettings to see only these variables
     */
    public interface PhotonVisionSettings {

        // PhotonVision camera mount location
        /*
        * Enter the camera location wrt the robot. Example camera mounted
        * facing forward (zero radians yaw), half meter forward of center, quarter meter up
        * from center, zero meters left of center, pitch pointing up 25 degrees. (camera pointing up
        * is "-""; down is "+"")
        */
        public static Transform3d robotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.25),
            new Rotation3d(0, Units.degreesToRadians(-25.), 0));
        
        // PhotonVision camera name; select the camera name set in PV dashboard
        // (at for example, photonvision.local:5800 or localhost:5800)
        public static String name = "Microsoft_LifeCam_HD-3000"; // "Arducam_OV9281_USB_Camera"
    }
}
