package frc.robot.drive;

public class DrivePower {

	private double leftPower;
	private double rightPower;

	/**
	 * Constructor to create the DrivePower object.
	 * @param leftPow set the left power
	 * @param rightPow set the right power
	 */
	public DrivePower(double leftPow, double rightPow) {
		leftPower = leftPow;
		rightPower = rightPow;
	}

	/**
	 * Multiplies the powers by a constant.
	 * @param scalar constant to multiply to the powers
	 * @return the new powers
	 */
	public DrivePower scale(double scalar) {
		leftPower *= scalar;
		rightPower *= scalar;
		return this;
	}

	/**
	 * Returns the left power.
	 * @return the left power
	 */
	public double getLeftPower() {
		return leftPower;
	}

	/**
	 * Returns the right power.
	 * @return the right power
	 */
	public double getRightPower() {
		return rightPower;
	}

	/**
	 * Sets the left power to the specified value.
	 * @param newLeftPower new power to which the left power will be set
	 */
	public void setLeftPower(double newLeftPower) {
		leftPower = newLeftPower;
	}

	/**
	 * Sets the right power to the specified value.
	 * @param newRightPower new power to which the right power will be set
	 */
	public void setRightPower(double newRightPower) {
		rightPower = newRightPower;
	}
}
