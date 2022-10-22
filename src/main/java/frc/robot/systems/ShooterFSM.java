package frc.robot.systems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ShooterFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		NO_BALL_STATE,
		WAIT_FOR_MOTOR,
		SHOOTER_READY,
		SHOOT
	}


	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax shooterMotor;
	private CANSparkMax transferMotor;
	private CANSparkMax intakeMotor;
	private ColorSensorV3 colorSensor;
	private Timer shooterTimer;
	//constants
	private static final double INTAKE_POWER = 0.5;
	private static final double TRANSFER_POWER = 1;
	private static final double MOTOR_RUN_POWER = 1;

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
		transferMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTER,
										CANSparkMax.MotorType.kBrushless);
		intakeMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTER,
										CANSparkMax.MotorType.kBrushless);
		colorSensor = new ColorSensorV3(Port.kOnboard);
		shooterTimer = new Timer();
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
		currentState = FSMState.NO_BALL_STATE;
		shooterMotor.set(0);
		intakeMotor.set(INTAKE_POWER);
		transferMotor.set(0);
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
			case NO_BALL_STATE:
				handleNoBallState(input);
				break;
			case WAIT_FOR_MOTOR:
				handleWaitForMotorState(input);
				break;
			case SHOOTER_READY:
				handleShooterReadyState(input);
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
			case NO_BALL_STATE:
				if (!colorSensorDetectsBall()) {
					return FSMState.NO_BALL_STATE;
				} else {
					shooterTimer.reset();
					shooterTimer.start();
					return FSMState.WAIT_FOR_MOTOR;
				}

			case WAIT_FOR_MOTOR:
				if (shooterTimer.get() <= 1) {
					return FSMState.WAIT_FOR_MOTOR;
				} else {
					return FSMState.SHOOTER_READY;
				}

			case SHOOTER_READY:
				if (!input.isShooterButtonPressed()) {
					return FSMState.SHOOT;
				}
				shooterTimer.reset();
				shooterTimer.start();
				return FSMState.SHOOT;
			case SHOOT:
				if (shooterTimer.get() <= 1) {
					return FSMState.SHOOT;
				}
				return FSMState.NO_BALL_STATE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	private boolean colorSensorDetectsBall() {
		//implement later
		return false;
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleNoBallState(TeleopInput input) {
		shooterMotor.set(0);
		intakeMotor.set(INTAKE_POWER);
		transferMotor.set(0);
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleWaitForMotorState(TeleopInput input) {
		shooterMotor.set(MOTOR_RUN_POWER);
		transferMotor.set(0);
		intakeMotor.set(0);
	}

	private void handleShooterReadyState(TeleopInput input) {
		shooterMotor.set(MOTOR_RUN_POWER);
		transferMotor.set(0);
		intakeMotor.set(0);
	}

	private void handleShootState(TeleopInput input) {
		shooterMotor.set(MOTOR_RUN_POWER);
		transferMotor.set(TRANSFER_POWER);
		intakeMotor.set(INTAKE_POWER);
	}

	private boolean shooterReady() {
		return true;
	}

	private void updateDashboard(TeleopInput input) {
		if (input != null) {
			SmartDashboard.putBoolean("Shooter Button Pressed", input.isShooterButtonPressed());
		} else {
			SmartDashboard.putBoolean("Shooter Button Pressed", false);
		}
		SmartDashboard.putNumber("Shooter Motor Power", shooterMotor.get());
		SmartDashboard.putNumber("Intermediate Motor Power", intakeMotor.get());
		SmartDashboard.putBoolean("Shooter Ready", shooterReady());
		SmartDashboard.putString("Current State", currentState + "");
	}
}

