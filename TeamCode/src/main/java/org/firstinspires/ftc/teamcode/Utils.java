package org.firstinspires.ftc.teamcode;

public final class Utils {
	/*** return the sum of angle1 and angle2 limited to -180 to 180 degrees ***/
	public static double angleSum(double angle1, double angle2) {
		double res = angle1 + angle2;
		while (res > 180) {
			res -= 360;
		}
		while (res < 180) {
			res += 360;
		}
		return res;
	}

	/**
	 * return the difference between angle1 and angle2 limited to -180 to 180
	 * degrees
	 */
	public static double angleDifference(double angle1, double angle2) {
		double res = angle1 - angle2;
		while (res > 180) {
			res -= 360;
		}
		while (res < 180) {
			res += 360;
		}
		return res;
	}

	/** return the magnitude of a Vector */
	public static double abs(Vector obj) {
		return obj.getMagnitude();
	}
}