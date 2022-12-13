package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {

	public static final int CAN_ID_SPARK_DRIVE_RIGHT = 5;
	public static final int CAN_ID_SPARK_DRIVE_LEFT = 6;

	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_MOTOR1 = 5;
	public static final int CAN_ID_SPARK_MOTOR2 = 2;
	public static final int CAN_ID_SPARK_MOTOR3 = 37;
	public static final int CAN_ID_SPARK_MOTOR4 = 32;

	//Intake
	public static final int INTAKE_MOTOR = 12;
	//Drive
	public static final int CAN_ID_SPARK_DRIVE_RIGHT1 = 4;
	public static final int CAN_ID_SPARK_DRIVE_RIGHT2 = 4;
	public static final int CAN_ID_SPARK_DRIVE_LEFT1 = 3;
	public static final int CAN_ID_SPARK_DRIVE_LEFT2 = 3;
	//Shooter
	public static final int CAN_ID_SPARK_SHOOTER = 40;
	public static final int TRANSFER_MOTOR_LEFT = 33;
	public static final int TRANSFER_MOTOR_RIGHT = 34;
	public static final int INTER = 11;
	public static final int CAN_ID_SPARK_PID = 5;
	//Climber
	public static final int CAN_ID_SPARK_CLIMBER = 2;
	public static final int CAN_ID_SPARK_CLIMBER_LEFT = 10;
	public static final int CAN_ID_SPARK_CLIMBER_RIGHT = 35;

	// Pneumatics channel numbers

	//Intake
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_EXTEND = 1;
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_RETRACT = 0;

	public static final int PCM_CHANNEL_INTAKE_CYLINDER_EXTEND2 = 3;
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_RETRACT2 = 2;
	//Climber
	public static final int PCM_CHANNEL_ARM_CYLINDER_EXTEND = 1;
	public static final int PCM_CHANNEL_ARM_CYLINDER_RETRACT = 0;

}
