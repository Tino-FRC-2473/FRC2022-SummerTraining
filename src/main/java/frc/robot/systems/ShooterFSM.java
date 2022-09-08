package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ShooterFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		INIT_STATE,
		TRANSFER_STATE,
		SHOOT
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax shooterMotor;
	private CANSparkMax interMotor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterFSM() {
		// Perform hardware init
		shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
										CANSparkMax.MotorType.kBrushless);
		interMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTER,
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
		currentState = FSMState.INIT_STATE;
		updateDashboard(null);
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
		updateDashboard(input);
		switch (currentState) {
			case INIT_STATE:
				handleInitState(input);
				break;

			case TRANSFER_STATE:
				handleTransferState(input);
				break;
			case SHOOT:
				handleShootState(input);
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
			case INIT_STATE:
				if (ballInIntermediate()) {
					return FSMState.TRANSFER_STATE;
				} else {
					return FSMState.INIT_STATE;
				}

			case TRANSFER_STATE:
				if (!shooterReady() || !input.isShooterButtonPressed()) {
					return FSMState.TRANSFER_STATE;
				} else {
					return FSMState.SHOOT;
				}

			case SHOOT:
				if (input.isShooterButtonPressed()) {
					return FSMState.SHOOT;
				}
				return FSMState.INIT_STATE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleInitState(TeleopInput input) {
		shooterMotor.set(0);
		interMotor.set(0);
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTransferState(TeleopInput input) {
		shooterMotor.set(MOTOR_RUN_POWER);
		interMotor.set(0);
	}

	private void handleShootState(TeleopInput input) {
		interMotor.set(MOTOR_RUN_POWER);
		shooterMotor.set(MOTOR_RUN_POWER);
	}

	private boolean shooterReady() {
		return true;
	}

	private boolean ballInIntermediate() {
		return true;
	}

	private void updateDashboard(TeleopInput input) {
		if (input != null) {
			SmartDashboard.putBoolean("Shooter Button Pressed", input.isShooterButtonPressed());
		} else {
			SmartDashboard.putBoolean("Shooter Button Pressed", false);
		}
		SmartDashboard.putNumber("Shooter Motor Power", shooterMotor.get());
		SmartDashboard.putNumber("Intermediate Motor Power", interMotor.get());
		SmartDashboard.putBoolean("Shooter Ready", shooterReady());
		SmartDashboard.putBoolean("Ball In Intermediate", ballInIntermediate());
		SmartDashboard.putString("Current State", currentState + "");
	}
}

