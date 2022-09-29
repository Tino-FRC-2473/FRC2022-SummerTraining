package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {
	private static LimeLight mLimeLight;

	private NetworkTable table;

	private NetworkTableEntry tx;
	private NetworkTableEntry ty;
	private NetworkTableEntry ta;

	private double[] defaultValue = new double[] {-2, -2, -2};



	/**
	 * LimeLight Constructor.
	 */
	public LimeLight() {
		table = NetworkTableInstance.getDefault().getTable("limelight");

		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
	}

	/**
	 * Updates limelight data by calling outputToShuffleboard.
	 */
	public void update() {
		if (getMotorPower() <= 1 && getMotorPower() >= -1) {
			SmartDashboard.getEntry("Shooting Power").setNumber(getMotorPower());
		}

		if (getTurningDirection() == -1) {
			SmartDashboard.getEntry("Turn Direction").setString("Left");
		} else if (getTurningDirection() == 0) {
			SmartDashboard.getEntry("Turn Direction").setString("Stay");
		} else if (getTurningDirection() == 1) {
			SmartDashboard.getEntry("Turn Direction").setString("Right");
		} else {
			SmartDashboard.getEntry("Turn Direction").setString("Invalid Entry");
		}

		SmartDashboard.getEntry("Distance To Hub").setNumber(getHubDistance());

		System.out.println("Distance: " + getHubDistance()
						+ " Direction: " + getTurningDirection()
						+ " Motor Power: " + getMotorPower());
	}

	/**
	 * Turns off Limelight.
	 */
	public void setOffLimelight() {
		table.getEntry("ledMode").setNumber(1);
	}

	/**
	 * Displays alliance color for CV.
	 * @param isRedAutoSelected is a red auto path selected
	 */
	public void setAllianceColor(boolean isRedAutoSelected) {
		if (isRedAutoSelected) {
			table.getEntry("llrobot").setDouble(0);
		} else {
			table.getEntry("llrobot").setDouble(1);
		}
	}

	/**
	 * Gets data about the ball position from network tables.
	 * @return an array of doubles in the following formath [distance, angle]
	 */
	public double[] getBallPosition() {
		return table.getEntry("llpython").getDoubleArray(defaultValue);
	}

	/**
	 * Gets data about the Hub Distance.
	 * @return a double
	 */
	public double getHubDistance() {
		return table.getEntry("llpython").getDoubleArray(defaultValue)[2];
	}

	/**
	 * Gets Turning Direction.
	 * @return a double
	 */
	public double getTurningDirection() {
		return table.getEntry("llpython").getDoubleArray(defaultValue)[1];
	}

	/**
	 * Gets data about the Shooter Motor Power.
	 * @return a double
	 */
	public double getMotorPower() {
		return table.getEntry("llpython").getDoubleArray(defaultValue)[0];
	}
}