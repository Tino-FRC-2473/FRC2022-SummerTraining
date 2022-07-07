package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj.SPI;
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class FSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		DRIVE_STATE,
		IDLE_STATE,
		TURNING_STATE,
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private AHRS gyro;


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
		gyro = new AHRS(SPI.Port.kMXP);
										
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
		currentState = FSMState.IDLE_STATE;
		gyro.reset();
		gyro.calibrate();

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
		if(input == null) return;
		switch (currentState) {
			case START_STATE:
				handleStartState(input);
				break;

			case DRIVE_STATE:
				handleDriveState(input);
				break;

			case IDLE_STATE:
				handleIdleState(input);
				break;
			
			case TURNING_STATE:
				handleTurningState(input);
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
			case START_STATE:
				if (Math.abs(input.getLeftJoystickY()) >= 0.2 || Math.abs(input.getRightJoystickY()) >= 0.2) {
					return FSMState.DRIVE_STATE;
				} else {
					return FSMState.START_STATE;
				}

			case DRIVE_STATE:
				return FSMState.DRIVE_STATE;

			case IDLE_STATE:
				if((gyro.getAngle() >= 175 && gyro.getAngle() <= 185) || (gyro.getAngle() <= -175 && gyro.getAngle() >= -185)) {
					return FSMState.TURNING_STATE;
				} else {
					return FSMState.IDLE_STATE;
				}
				
			case TURNING_STATE:
				return FSMState.TURNING_STATE;

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
	private void handleStartState(TeleopInput input) {
		if(input == null) return;
		leftMotor.set(0);
		rightMotor.set(0);
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleDriveState(TeleopInput input) {
		if(input == null) return;
		
		// SlewRateLimiter filterL = new SlewRateLimiter(0.5); //limit the rate of change to 0.5
		// SlewRateLimiter filterR = new SlewRateLimiter(0.5); //limit the rate of change to 0.5
		// leftMotor.set(filterL.calculate(-input.getLeftJoystickY()));
		// rightMotor.set(filterR.calculate(input.getRightJoystickY()));

		// leftMotor.set(input.getLeftJoystickY());
		// rightMotor.set(input.getRightJoystickY());

		long prevTime = System.currentTimeMillis();
		double currentSpeedL = input.getLeftJoystickY();
		double currentSpeedR = input.getRightJoystickY();
		System.out.println("left speed: " + currentSpeedL);
		System.out.println("right speed: " + currentSpeedR);
		double change = 0.2;
		double targetSpeed = 0.5;

		
			

		//limiting left motor acceleration
		if (Math.abs(currentSpeedL) < targetSpeed)
		{
			currentSpeedL += change * (currentSpeedL/Math.abs(currentSpeedL));
			if (Math.abs(currentSpeedL) > targetSpeed) currentSpeedL = targetSpeed;

			System.out.println("left speed: " + currentSpeedL);
			System.out.println("right speed: " + currentSpeedR);
		}		
		if (Math.abs(currentSpeedL) > targetSpeed)
		{
			currentSpeedL -= change * (currentSpeedL/Math.abs(currentSpeedL));
			if (Math.abs(currentSpeedL) < targetSpeed) currentSpeedL = targetSpeed;

			System.out.println("left speed: " + currentSpeedL);
			System.out.println("right speed: " + currentSpeedR);
		}
		leftMotor.set(currentSpeedL);

		//limiting right motor acceleration
		if (Math.abs(currentSpeedR) < targetSpeed)
		{
			currentSpeedR += change * (currentSpeedR/Math.abs(currentSpeedR));
			if (Math.abs(currentSpeedR) > targetSpeed) currentSpeedR = targetSpeed;

			System.out.println("left speed: " + currentSpeedL);
			System.out.println("right speed: " + currentSpeedR);
		}	
		if (Math.abs(currentSpeedR) > targetSpeed)
		{
			currentSpeedR -= change * (currentSpeedR/Math.abs(currentSpeedR));
			if (Math.abs(currentSpeedR) < targetSpeed) currentSpeedR = targetSpeed;

			System.out.println("left speed: " + currentSpeedL);
			System.out.println("right speed: " + currentSpeedR);
		}
		rightMotor.set(currentSpeedR);
				
			
		
	}

	private void handleIdleState(TeleopInput input) {
		if(input == null) 
			leftMotor.set(0);
			rightMotor.set(0);
	}

	private void handleTurningState(TeleopInput input) {
		if(input == null) 
			leftMotor.set(0.5);
			rightMotor.set(-0.5);
	}
}
