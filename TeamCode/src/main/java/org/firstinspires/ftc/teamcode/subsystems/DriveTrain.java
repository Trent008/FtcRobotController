package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constants.DTConstants;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.Vector;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public final class DriveTrain {
	// for storing the current rate setpoints
	private Vector currentFieldRate = new Vector();
	private double currentTurnRate = 0;
	// for storing the target rates
	private Vector targetFieldRate;
	private double targetTurnRate;
	// current position on the field relative to the starting position
	private Vector currentFieldPosition = DTConstants.startingPosition;
	// current angle on the field
	private double currentFieldAngle;
	// reported angle of the imu relative to the start of the program
	private double currentIMUAngle;

	// gyro (imu) object
	private BNO055IMU imu;

	// used to find the time between set calls to achieve consistent acceleration
	private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

	// construct all mecanum wheel objects
	private MecanumWheel leftFront = new MecanumWheel("leftFront", -1, -1, -1);
	private MecanumWheel leftBack = new MecanumWheel("leftBack", 1, -1, -1);
	private MecanumWheel rightFront = new MecanumWheel("rightFront", -1, 1, -1);
	private MecanumWheel rightBack = new MecanumWheel("rightBack", 1, 1, -1);
	private MecanumWheel[] wheels = { leftFront, leftBack, rightFront, rightBack };

	public void initialize(HardwareMap hwMap) {
		// initialize all mecanum wheels
		for (MecanumWheel wheel : wheels) {	wheel.initialize(hwMap); }

		// gyro (imu) parameters
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json";

		// initialize imu
		imu = hwMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);


		timer.reset();
	}

	/**
	 * drive the mecanum chassis
	 * 
	 * @param targetFieldRate field-centric rate of the chassis
	 * @param targetTurnRate turning rate of the chassis
	 * @param useAcceleration should the chassis change velocity smoothly
	 */
	public void set(Vector targetFieldRate, double targetTurnRate, boolean useAcceleration) {
		// store the target rates
		this.targetFieldRate = targetFieldRate;
		this.targetTurnRate = targetTurnRate;
		// find gyro angle
		currentIMUAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
		// set the current field angle to the gyro angle + the starting angle
		currentFieldAngle = Utils.angleSum(currentIMUAngle, DTConstants.startingAngle);
		// modify the target rates to make them physically achievable
		normalizeWheelSpeeds();
		// apply a slew rate if using acceleration
		if (useAcceleration) {
			applySlewRate(targetFieldRate, targetTurnRate);
		} else { // otherwise, set the drive rates directly from the input
			currentFieldRate = targetFieldRate;
			currentTurnRate = targetTurnRate;
		}
		// store the robot's change in position since last set() call
        Vector fieldPositionChange = new Vector();
        // drive the modules and average the module position changes
        for (MecanumWheel wheel : wheels)
        {
            wheel.drive(currentFieldRate.getRotatedCW(-currentFieldAngle), currentTurnRate);
            fieldPositionChange.add(wheel.getPositionChangeVector());
        }
        // average the position change of all four mecanum wheels
        fieldPositionChange.divide(4);
        // field-orient the position change vector
        fieldPositionChange.rotateCW(currentFieldAngle);
        // add the change in position over this cycle to the running total
        currentFieldPosition.add(fieldPositionChange);
	}

	/**
	 * drive toward a pose on the field from the current pose
	 * 
	 * @param targetPosition    position to drive to
	 * @param targetAngle       angle to go to
	 * @param positionTolerance how close must the position be to the target to return true
	 * @param angleTolerance    how close must the angle be to the target to return true
	 * @return true when current angle and position are within tolerance
	 */
	public boolean driveToward(final Vector targetPosition, final double targetAngle, final double positionTolerance,
			final double angleTolerance) {
		// find the position and angle errors
		Vector positionError = targetPosition.getSubtracted(currentFieldPosition);
		double angleError = Utils.angleDifference(currentFieldAngle, targetAngle);

		// calculate outputs based on error
		Vector positionPIDOutput = positionError.getScaled(DTConstants.autoPositionP);
		double anglePIDOutput = angleError * DTConstants.autoAngleP;

		// scale outputs down to max
		if (Utils.abs(positionPIDOutput) > DTConstants.autoMaxDriveRate) {
			positionPIDOutput.getScaled(DTConstants.autoMaxDriveRate / Utils.abs(positionPIDOutput));
		}
		if (Math.abs(anglePIDOutput) > DTConstants.autoMaxTurnRate) {
			anglePIDOutput *= DTConstants.autoMaxTurnRate / Math.abs(anglePIDOutput);
		}

		// set the drivetrain to move toward targets
		set(positionPIDOutput, anglePIDOutput, false);

		// return whether or not the robot has reached the position and angle targets
		return (Utils.abs(positionError) < positionTolerance) && (Math.abs(angleError) < angleTolerance);
	}

	/**
	 * increment current field and turn rates toward the target rates
	 */
	private void applySlewRate(final Vector targetFieldRate, final double targetTurnRate) {
		Vector fieldRateError = targetFieldRate.getSubtracted(currentFieldRate).getScaled(0.5);
		// calculate slew-rate based on how much time has passed
		double slewRate = timer.milliseconds() / DTConstants.timeToMax;
		timer.reset();
		if (Utils.abs(fieldRateError) > slewRate) {
			// increment toward target at the slew rate
			currentFieldRate.add(fieldRateError.getScaled(slewRate / Utils.abs(fieldRateError)));

		} else {
			currentFieldRate = targetFieldRate;
		}
		double turnRateError = (targetTurnRate - currentTurnRate);
		if (Math.abs(turnRateError) > slewRate) {
			currentTurnRate += slewRate / Math.abs(turnRateError);
		} else {
			currentTurnRate = targetTurnRate;
		}
	}

	/**
	 * limits the target rates to achievable values while still allowing full speed
	 * in every direction
	 */
	private void normalizeWheelSpeeds() {
		// robot-orient the target field rate
		this.targetFieldRate.rotateCW(-currentFieldAngle);
		// find the fastest wheel speed
		double fastestSpeed = 1;
		for (MecanumWheel wheel : wheels) {
			double wheelSpeed = Math.abs(wheel.getVelocity(targetFieldRate, targetTurnRate));
			if (wheelSpeed > fastestSpeed) {
				fastestSpeed = wheelSpeed;
			}
		}
		// limit the target rates so that the wheel speeds won't ecxeed 1
		targetFieldRate.divide(fastestSpeed);
		targetTurnRate /= fastestSpeed;
		// make the target field rate field-centric again
		targetFieldRate.rotateCW(currentFieldAngle);
	}

}