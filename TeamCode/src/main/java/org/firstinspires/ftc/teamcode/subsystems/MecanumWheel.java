package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants.MWConstants;
import org.firstinspires.ftc.teamcode.Vector;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumWheel {
    private final double xMultiplier;
    private final double yMultiplier;
    private final double rMultiplier;
    private double lastPosition = 0;

    HardwareMap hwMap = null;
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

    public void initialize(HardwareMap hwMap) {
        this.hwMap = hwMap;
        motor = this.hwMap.get(DcMotor.class, motorName);
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
        motor.setPower(getVelocity(driveRate, turnRate));
    }

    public Vector getPositionChangeVector() {
        // find the current position in encoder counts
        double currentPosition = motor.getCurrentPosition();
        // find the position change since last call and convert to inches
        double positionChange = (currentPosition - lastPosition) * MWConstants.inchesPerEncoder;
        // save the current position for later
        lastPosition = currentPosition;
        // find the robot centric position change of the wheel
        Vector positionChangeVector = new Vector(xMultiplier, yMultiplier);
        positionChangeVector.scale(positionChange);
        return positionChangeVector;
    }
}
