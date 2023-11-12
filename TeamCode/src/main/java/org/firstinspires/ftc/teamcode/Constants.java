package org.firstinspires.ftc.teamcode;

public final class Constants {
    /**
     * drivetrain constants
     */
    public static final class DTConstants {
        /** milliseconds to max speed */
        public static final double timeToMax = 800.0;
        // max drive rate for autonomous
        public static final double autoMaxDriveRate = 0.5;
        // max rotation rate for autonomous
        public static final double autoMaxTurnRate = 0.5;
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
        public static final double wheelDiameterInches = 100.0 / 25.4;
        // inches per drive motor encoder (change as needed)
        public static final double inchesPerEncoder = wheelDiameterInches * Math.PI / 537.7;
    }
}
