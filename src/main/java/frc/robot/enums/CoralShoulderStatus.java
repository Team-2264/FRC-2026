package frc.robot.enums;

import frc.robot.Constants;

/**
 * Enum for the Coral Shoulder status. Represents the reef level the robot is aiming for.
 * 
 */
public enum CoralShoulderStatus {
    HOME, L1, L2, L3, L4;

    /**
     * Returns the angle the shoulder should be at for the given state.
     * 
     * @return The angle the shoulder should be at in rotations.
     */
    public double shoulderAngle() {
        double degrees = switch(this){
            case HOME -> 55;
            case L1 -> 0; // TODO: set angle
            case L2 -> -20;
            case L3 -> -20;
            case L4 -> -17;
    
        };

        return degrees / 360.0;
    
    }

    /**
     * Returns the height the shoulder should move for the given state.
     * 
     * @return The goal height based on the reef level.
     */
    public double shoulderHeight(){
        double height = switch(this){
            case HOME -> 0;
            case L1 -> Constants.gamePieces.L1Height;
            case L2 -> Constants.gamePieces.L2Height;
            case L3 -> Constants.gamePieces.L3Height;
            case L4 -> Constants.gamePieces.L4Height;
        };

        return height;
    }
}
