package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int INTAKE_MOTOR = 1;
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 2;
	public static final int CAN_ID_SPARK_DRIVE_FRONT_LEFT = 3;
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 4;
	public static final int CAN_ID_SPARK_SHOOTER = 5;

	// Pneumatics channel numbers
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_EXTEND = 1;
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_RETRACT = 2;
}
