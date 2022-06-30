package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class TeleOp {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		AUTO,
        TELEOP
	}

	private static final float MOTOR_RUN_POWER = 0.25f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax frontLeftMotor;
    private CANSparkMax frontRightMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TeleOp() {
		// Perform hardware init
        frontLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT, CANSparkMax.MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT, CANSparkMax.MotorType.kBrushless);

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
		currentState = FSMState.TELEOP;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case AUTO:
				handleAuto(input);
				break;

            case TELEOP:
				handleTeleop(input);
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
		switch (currentState) {
			case AUTO:
				if (input != null) {
					return FSMState.TELEOP;
				} else {
					return FSMState.AUTO;
				}

			case TELEOP:
				return FSMState.TELEOP;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in TELEOP.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleop(TeleopInput input) {
		if (input == null) return;

		double wheelThreshold = 0.5;
		double throttleThreshold = 0.1;

		if (input.getLeftJoystickX() > -wheelThreshold && input.getLeftJoystickX() < wheelThreshold){
            // Wheel Straight
			if (input.getRightJoystickY() > throttleThreshold){
				frontLeftMotor.set(MOTOR_RUN_POWER);
				frontRightMotor.set(MOTOR_RUN_POWER);
			} else if (input.getRightJoystickY() < -throttleThreshold){
				frontLeftMotor.set(-MOTOR_RUN_POWER);
				frontRightMotor.set(-MOTOR_RUN_POWER);
			} else {
				frontLeftMotor.set(0);
				frontRightMotor.set(0);
			}
        } else if (input.getLeftJoystickX() >= wheelThreshold){
			// Wheel right
            if (input.getRightJoystickY() > throttleThreshold){
				frontLeftMotor.set(MOTOR_RUN_POWER);
				frontRightMotor.set(MOTOR_RUN_POWER/2);
			} else {
				frontLeftMotor.set(0);
				frontRightMotor.set(0);
			}
        } else if (input.getLeftJoystickX() <= -wheelThreshold){
			// Wheel left
            if (input.getRightJoystickY() > throttleThreshold){
				frontLeftMotor.set(MOTOR_RUN_POWER/2);
				frontRightMotor.set(MOTOR_RUN_POWER);
			} else {
				frontLeftMotor.set(0);
				frontRightMotor.set(0);
			}
        } 
	}
	/**
	 * Handle behavior in AUTO.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleAuto(TeleopInput input) {
		
	}
}
