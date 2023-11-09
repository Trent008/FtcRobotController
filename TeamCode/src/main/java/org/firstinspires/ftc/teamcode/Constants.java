package frc.robot;

public final class Constants {
    /**
     * drivetrain constants
     */
    public static final class DTConstants {
        // amount to change the robot velocity per teleop cycle
        public static final double slewRate = 0.035;
        // max drive rate for autonomous
        public static final double autoMaxDriveRate = 0.2;
        // max rotation rate for autonomous
        public static final double autoMaxTurnRate = 0.2;
        // proportional constant for autonomous position error
        public static final double autoPositionP = 0.01;
        // proportional constant for autonomous angle error
        public static final double autoAngleP = 0.005;

        // robot starting position on the field
        public static final Vector startingPosition = new Vector(0, 0);
        // robot starting angle on the field
        public static final double startingAngle = 0;
    }

    /**
     * mecanum wheel constants
     */
    public static final class MWConstants {
        // inches per drive motor encoder (change as needed)
        public static final double mecanumWheelInchesPerEncoder = 0.1;
    }
}
