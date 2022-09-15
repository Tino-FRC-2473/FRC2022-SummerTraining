package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
	private static LimeLight mLimeLight;

	private NetworkTable table;

	private NetworkTableEntry tx;
	private NetworkTableEntry ty;
	private NetworkTableEntry ta;

	private double[] defaultValue = new double[] {-1, -1};

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
	 * Get the instance of LimeLight. Makes new instance if null.
	 * @return The LimeLight instance
	 */
	public static LimeLight getInstance() {
		if (mLimeLight == null) {
			mLimeLight = new LimeLight();
		}
		return mLimeLight;
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
	 * Gets data about the Hub Distance
	 * @return a double
	 */
    public double getHubDistance() {
        return table.getEntry("Distance").getDouble(0);
    }
    
    /**
	 * Gets Turning Direction
	 * @return a double
	 */
    public double getTurningDirection() {
        return table.getEntry("Turning Direction").getDouble(0);
    }
    
    /**
	 * Gets data about the Shooter Motor Power
	 * @return a double
	 */
    public double getMotorPower() {
        return table.getEntry("Motor Power").getDouble(0);
    }
}

 
