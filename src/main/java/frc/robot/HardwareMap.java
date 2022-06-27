package frc.robot;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_DRIVE_FRONT_RIGHT = 1;
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 2;
	public static final int CAN_ID_SPARK_DRIVE_FRONT_LEFT = 3;
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 4;
	public static final int CAN_ID_SPARK_SHOOTER = 5;

	// Pneumatics channel numbers
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_FORWARD = 1;
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_REVERSE = 2;

	private CANSparkMax driveMotor1;
	private CANSparkMax driveMotor2;


	public HardwareMap() {
				// Perform hardware init
				driveMotor1 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
				CANSparkMax.MotorType.kBrushless);
				driveMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
				CANSparkMax.MotorType.kBrushless);				
	}

	public void setSpeedMotor1(double speed) {
		driveMotor1.set(speed);
	}

	public void setSpeedMotor2(double speed) {
		driveMotor2.set(speed);
	}

	public void setStartState() {
		driveMotor1.set(0);
		driveMotor2.set(0);
	}

	public void setRunPower(float runPower) {
		driveMotor1.set(runPower);
		driveMotor2.set(runPower);
	} 
}



