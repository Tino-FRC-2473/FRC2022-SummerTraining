package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {

	// ID numbers for devices on the CAN bus

	//Intake
	public static final int INTAKE_MOTOR = 10;
	//Drive
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 2;
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 3;
	//Shooter
	public static final int CAN_ID_SPARK_SHOOTER = 5;
	//Climber
	public static final int CAN_ID_SPARK_CLIMBER = 4;
	public static final int LIMIT_SWITCH_ID_FIRST = 0;
	public static final int LIMIT_SWITCH_ID_SECOND = 1;
	// Pneumatics channel numbers

	//Intake
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_EXTEND = 1;
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_RETRACT = 0;
	//Climber
	public static final int PCM_CHANNEL_ARM_CYLINDER_EXTEND = 2;
	public static final int PCM_CHANNEL_ARM_CYLINDER_RETRACT = 3;

}
