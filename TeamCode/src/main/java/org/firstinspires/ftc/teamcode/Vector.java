package org.firstinspires.ftc.teamcode;

/*** stores a 2D vector for doing vector math ***/
public class Vector {
	public double x;
	public double y;

	/*** construct a 2D vector ***/
	public Vector(final double x, final double y) {
		this.x = x;
		this.y = y;
	}

	public Vector() {
		this(0, 0);
	}

	/*** add another vector to this vector ***/
	public void add(final Vector other) {
		x += other.x;
		y += other.y;
	}

	/*** get the result of adding another vector to this one ***/
	public Vector getAdded(final Vector other) {
		final Vector result = this;
		result.add(other);
		return result;
	}

	/*** subtract another vector from this vector ***/
	public void subtract(final Vector other) {
		x -= other.x;
		y -= other.y;
	}

	/*** get the result of subtracting another vector from this one ***/
	public Vector getSubtracted(final Vector other) {
		final Vector result = this;
		result.subtract(other);
		return result;
	}

	/*** scale this vector by a constant ***/
	public void scale(final double k) {
		x *= k;
		y *= k;
	}

	/*** get the result of scaling this vector by a constant ***/
	public Vector getScaled(final double k) {
		return new Vector(x * k, y * k);
	}

	/*** divide this vector by a constant ***/
	public void divide(final double k) {
		x /= k;
		y /= k;
	}

	/*** get the result of scaling this vector by a constant ***/
	public Vector getDivided(final double k) {
		return new Vector(x / k, y / k);
	}

	/*** get the magnitude (length) of this vector ***/
	public double getMagnitude() {
		return Math.hypot(x, y);
	}

	/*** convert from degrees to radians ***/
	public double radians(final double degrees) {
		return degrees * Math.PI / 180;
	}

	/*** convert from radians to degrees ***/
	public double degrees(final double radians) {
		return radians * 180 / Math.PI;
	}

	/*** return this vector's angle (-180 to 180 degrees) ***/
	public double getAngle() {
		return degrees(Math.atan2(x, y));
	}

	/*** rotate this vector clockwise by the given angle ***/
	public void rotateCW(double angle) {
		angle = radians(angle);
		double newx = x * Math.cos(angle) + y * Math.sin(angle);
		double newy = y * Math.cos(angle) - x * Math.sin(angle);
		x = newx;
		y = newy;
	}

	/*** return this vector rotated by the given angle ***/
	public Vector getRotatedCW(double angle) {
		angle = radians(angle);
		double newx = x * Math.cos(angle) + y * Math.sin(angle);
		double newy = y * Math.cos(angle) - x * Math.sin(angle);
		return new Vector(newx, newy);
	}

	/*** reset the vector to (0, 0) ***/
	public void reset() {
		x = 0;
		y = 0;
	}
}