package frc.robot;

import frc.robot.Constants.MWConstants;

public class MecanumWheel {
    private double xMultiplier;
    private double yMultiplier;
    private double rMultiplier;
    private double lastPosition = 0;

    DcMotor motor;
    String motorName;

    /**
     * construct a mecanum wheel object
     * @param xMultiplier how to scale the x input for the wheel speed
     * @param yMultiplier how to scale the y input for the wheel speed
     * @param rMultiplier how to scale the rotation input for the wheel speed
     */
    public MecanumWheel(String motorName, double xMultiplier, double yMultiplier, double rMultiplier) {
        this.motorName = motorName;
        this.xMultiplier = xMultiplier;
        this.yMultiplier = yMultiplier;
        this.rMultiplier = rMultiplier;
    }

    public void initialize() {
        this.motor = hardwareMap.get(DcMotor.class, motorName);
    }

    /**
     * get this wheel's velocity
     * @param driveRate robot-centric drive rate command
     * @param turnRate turn rate command
     */
    public double getVelocity(Vector driveRate, double turnRate) {
        return driveRate.x * xMultiplier + driveRate.y * yMultiplier + turnRate * rMultiplier;
    }

    /**
     * make this wheel drive
     * @param driveRate robot-centric drive rate command
     * @param turnRate turn rate command
     */
    public void drive(Vector driveRate, double turnRate) {
        getVelocity(driveRate, turnRate); // todo: use this value to drive your motor
    }

    public Vector getPositionChangeVector() {
        // find the current position in encoder counts
        double currentPosition = 0; // todo: actually find the current position of the motor
        // find the position change since last call and convert to inches
        double positionChange = (currentPosition - lastPosition) * MWConstants.mecanumWheelInchesPerEncoder;
        // save the current position for later
        lastPosition = currentPosition;
        // find the robot centric position change of the wheel
        Vector positionChangeVector = new Vector(xMultiplier, yMultiplier);
        positionChangeVector.scale(positionChange);
        return positionChangeVector;
    }
}
