package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SPI;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class FSMSystem {

	private AHRS gyro;

	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELE_STATE_2_MOTOR_DRIVE,
		TELE_STATE_MECANUM
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;

	private CANSparkMax bottomLeftMotorMecanum;
	private CANSparkMax bottomRightMotorMecanum;
	private CANSparkMax topLeftMotorMecanum;
	private CANSparkMax topRightMotorMecanum;

	private double bottomLeftMotorMecanumPower;
	private double bottomRightMotorMecanumPower;
	private double topLeftMotorMecanumPower;
	private double topRightMotorMecanumPower;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT,
										CANSparkMax.MotorType.kBrushless);

		topLeftMotorMecanum = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_TOP_LEFT,
										CANSparkMax.MotorType.kBrushless);
		bottomLeftMotorMecanum = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BOTTOM_LEFT,
										CANSparkMax.MotorType.kBrushless);
		topLeftMotorMecanum = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_TOP_LEFT,
										CANSparkMax.MotorType.kBrushless);
		topRightMotorMecanum = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_TOP_RIGHT,
										CANSparkMax.MotorType.kBrushless);

		// Reset state machine
		reset();

		gyro = new AHRS(SPI.Port.kMXP);
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

		bottomLeftMotorMecanum.set(0);
		bottomRightMotorMecanum.set(0);
		topLeftMotorMecanum.set(0);
		topRightMotorMecanum.set(0);

		currentState = FSMState.TELE_STATE_MECANUM;

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
			case TELE_STATE_2_MOTOR_DRIVE:
				handleTeleOp2MotorState(input);
				break;
			
			case TELE_STATE_MECANUM:
				handleTeleOpMecanum(input);
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

			case TELE_STATE_2_MOTOR_DRIVE:
				return FSMState.TELE_STATE_2_MOTOR_DRIVE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in TELE_STATE_2_MOTOR_DRIVE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOp2MotorState(TeleopInput input) {
		if(input == null) {
			return;
		}
		
		leftMotor.set((input.getLeftJoystickY()));
		rightMotor.set(-(input.getRightJoystickY()));

		// System.out.println(gyro.getAngle());
	}

	/**
	 * Handle behavior in TELE_STATE_MECANUM.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOpMecanum(TeleopInput input) {
		
	}
}