package frc.robot.systems;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ShooterTester {


	public enum FSMState {
		IDLE,
		RUNNING
	}

	private static final float INTAKE_POWER = 0.1f;
	private static final float PREP_MOTOR_POWER = 0.3f;
	private static final float SHOOTER_POWER = 0.8f;

	private FSMState currentState;

	private CANSparkMax interMotor1;
	private CANSparkMax interMotor2;
	private CANSparkMax prepMotor;
	private CANSparkMax shooterMotor;

	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterTester() {
		interMotor1 = new CANSparkMax(HardwareMap.TRANSFER_MOTOR_LEFT,
										CANSparkMax.MotorType.kBrushless);
		interMotor2 = new CANSparkMax(HardwareMap.TRANSFER_MOTOR_RIGHT,
										CANSparkMax.MotorType.kBrushless);
		prepMotor = new CANSparkMax(HardwareMap.INTER,
										CANSparkMax.MotorType.kBrushless);
		shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
										CANSparkMax.MotorType.kBrushed);
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.IDLE;
		// Call one tick of update to ensure outputs reflect start state
		// updateDashboard(null);
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case IDLE:
				interMotor1.set(0);
				interMotor2.set(0);
				prepMotor.set(0);
				shooterMotor.set(0);
				break;
			case RUNNING:
				interMotor1.set(INTAKE_POWER);
				interMotor2.set(-INTAKE_POWER);
				prepMotor.set(PREP_MOTOR_POWER);
				shooterMotor.set(SHOOTER_POWER);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());

		}

		currentState = nextState(input);
	}

	private FSMState nextState(TeleopInput input) {
		if (input == null) {
			return FSMState.IDLE;
		}
		if (input.isIntakeButtonPressed()) {
			return FSMState.RUNNING;
		}
		return FSMState.IDLE;
	}
}
