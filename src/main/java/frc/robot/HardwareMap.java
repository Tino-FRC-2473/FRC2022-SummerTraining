package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {

	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_MOTOR1 = 5;
	public static final int CAN_ID_SPARK_MOTOR2 = 2;
	public static final int CAN_ID_SPARK_MOTOR3 = 37;
	public static final int CAN_ID_SPARK_MOTOR4 = 32;

	//Intake
	public static final int INTAKE_MOTOR = 12;
	//Drive
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 4;
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 3;
	//Shooter
	public static final int CAN_ID_SPARK_SHOOTER = 32;
	public static final int TRANSFER_MOTOR_LEFT = 33;
	public static final int TRANSFER_MOTOR_RIGHT = 34;
	public static final int INTER = 11;
	//Climber
	public static final int CAN_ID_SPARK_CLIMBER = 2;

	// Pneumatics channel numbers

	//Intake - for pneumatics: Odds are Forward, Evens are Retract - PKS + Sid
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_EXTEND = 1; // kForward
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_RETRACT = 0; // kRetract

	public static final int PCM_CHANNEL_INTAKE_CYLINDER_EXTEND2 = 3; // kForward
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_RETRACT2 = 2; // kRetract
	//Climber
	public static final int PCM_CHANNEL_ARM_CYLINDER_EXTEND = 1;
	public static final int PCM_CHANNEL_ARM_CYLINDER_RETRACT = 0;

}
