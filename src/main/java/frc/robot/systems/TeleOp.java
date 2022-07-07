
package frc.robot.systems;
// WPILib Imports
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class TeleOp {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP, AUTO
	}

	private static final float MOTOR_RUN_POWER = 0.1f;
	private static final float TURN_DEG = 175f;
	private static final float ACCEL_CONSTANT = 10f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private double currLpower = 0;
	private double currRpower = 0;
	private AHRS gyro = new AHRS(SPI.Port.kMXP);
	private boolean rotating = true;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TeleOp() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
										CANSparkMax.MotorType.kBrushless);
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
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
		currentState = FSMState.AUTO;
		gyro.reset();
		gyro.calibrate();
		// Call one tick of update to ensure outputs reflect start state
		//update(null);
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case TELEOP:
				handle(input);
				break;
			case AUTO:
				handleAuto(null);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}
	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		if (input == null) {
			return FSMState.AUTO;
		} else {
			return FSMState.TELEOP;
		}
	}
	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handle(TeleopInput input) {
		//arcade drive
		double desiredLpower = input.getLeftJoystickY() - input.getRightJoystickX();
		double desiredRpower =  -input.getLeftJoystickY() - input.getRightJoystickX();
		if (desiredLpower > 1) {
			desiredLpower = 1;
		} else if (desiredLpower < -1) {
			desiredLpower = -1;
		}
		if (desiredRpower > 1) {
			desiredRpower = 1;
		} else if (desiredRpower < -1) {
			desiredRpower = -1;
		}
		currLpower += (desiredLpower - currLpower) / ACCEL_CONSTANT;
		currRpower += (desiredRpower - currRpower) / ACCEL_CONSTANT;
		leftMotor.set(currLpower);
		rightMotor.set(currRpower);
		//tank drive
		//leftMotor.set(-input.getLeftJoystickY());
		//rightMotor.set(input.getRightJoystickY());
	}
	private void handleAuto(TeleopInput input) {
		if (rotating) {
			if (gyro.getAngle() < TURN_DEG) {
				//turn CW
				leftMotor.set(-MOTOR_RUN_POWER);
				rightMotor.set(-MOTOR_RUN_POWER);
			} else {
				leftMotor.set(0);
				rightMotor.set(0);
				rotating = false;
			}
		}
	}
}
