package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class LimeLight {

	private PhotonCamera camera = new PhotonCamera("photonvision");
	private NetworkTable table;

	private NetworkTableEntry tx;
	private NetworkTableEntry ty;
	private NetworkTableEntry ta;

	private double[] defaultValue = new double[] {-1, -1, -1};

	private static final double CAMERA_ANGLE = Math.toRadians(26); //RADIANS
	private static final double HUB_HEIGHT = 26; //METERS
	private static final double CAMERA_HEIGHT = 26; //METERS
	private static final double SIGMOID_CONST1 = 0.5;
	private static final double SIGMOID_CONST2 = 10;
	private static final double INVALID_RETURN = -2;


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
	 * Checks if limelight is tracking any targets.
	 * @return If limelight has any valid targets (0 or 1)
	 */
	public boolean hasValidTargets() {
		return table.getEntry("tv").getDouble(0) == 1;
	}

	/**
	 * Returns horizontal offset from crosshair to target.
	 * @return horizontal offset from crosshair to target
	 */
	public double getXAngle() {
		return tx.getDouble(0);
	}

	/**
	 * Returns vertical offset from crosshair to target.
	 * @return vertical offset from crosshair to target
	 */
	public double getYAngle() {
		return ty.getDouble(0);
	}

	/**
	 * Returns area of vision tracking box.
	 * @return Target Area (0% to 100% of image)
	 */
	public double getArea() {
		return ta.getDouble(0);
	}

	/**
	 * Returns rotation of object.
	 * @return Skew or rotation of object (-90 to 0 degrees)
	 */
	public double getSkew() {
		return table.getEntry("ts").getDouble(0);
	}

	/**
	 * X-coordinates of the tracked box.
	 * @return Number array of corner x-coordinates
	 */
	public double[] getXCorners() {
		return table.getEntry("tcornx").getDoubleArray(new double[] {0, 0, 0, 0});
	}

	/**
	 * Y-coordinates of the tracked box.
	 * @return Number array of corner y-coordinates
	 */
	public double[] getYCorners() {
		return table.getEntry("tcorny").getDoubleArray(new double[] {0, 0, 0, 0});
	}


	/**
	 * Updates limelight data by calling outputToShuffleboard.
	 */
	public void update() {

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
		var result = camera.getLatestResult();
		if (result.hasTargets()) {
			return PhotonUtils.calculateDistanceToTargetMeters(
								CAMERA_HEIGHT,
								HUB_HEIGHT,
								CAMERA_ANGLE,
								Math.toRadians(result.getBestTarget().getPitch()));
		}
		return -1;
	}

	/**
	 * Gets Turning Direction.
	 * @return a double
	 */
	public double getTurningPower() {
		var result = camera.getLatestResult();
		if (!result.hasTargets()) {
			return INVALID_RETURN;
		}
		double angle = result.getBestTarget().getYaw();
		return (1 / (1 + Math.exp(-angle / SIGMOID_CONST2))) - SIGMOID_CONST1;
	}

	/**
	 * Which direction to turn for aligning with the ball.
	 * @return a double
	 */
	public double getBallTurnDirection() {
		return table.getEntry("llpython").getDoubleArray(defaultValue)[0];
	}
	/**
	 * Whether we are close enough to intake the ball or not.
	 * @return a double
	 */
	public double getIntakeStatus() {
		return table.getEntry("llpython").getDoubleArray(defaultValue)[1];
	}
}
