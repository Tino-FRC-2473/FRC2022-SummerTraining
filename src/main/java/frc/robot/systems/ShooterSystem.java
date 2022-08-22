package frc.robot.systems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.HardwareMap;

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;

public class ShooterSystem {


	// FSM state definitions
	public enum FSMState {
		INTAKING,
		BALL_HOLDING,
		SHOOTING
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private DigitalInput magneticProxLeft = new DigitalInput(0);
	private DigitalInput magneticProxMid = new DigitalInput(1);
	private DigitalInput magneticProxRight = new DigitalInput(2);

	private CANSparkMax shooterLeft;
	private CANSparkMax shooterRight;
	private CANSparkMax shooterDriver;

	private boolean doneShooting = false;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ShooterSystem() {

		// Set Current State:
		currentState = FSMState.BALL_HOLDING;

		// Perform hardware init
		shooterLeft = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER_MOTOR_LEFT,
										CANSparkMax.MotorType.kBrushless);
		shooterRight = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER_MOTOR_RIGHT,
										CANSparkMax.MotorType.kBrushless);
		shooterDriver = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER_DRIVER,
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

		currentState = FSMState.BALL_HOLDING;

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
			case INTAKING:
				handleIntakingState(input);
				break;

			case BALL_HOLDING:
				handleBallHoldingState(input);
				break;

			case SHOOTING:
				handleShootingState(input);
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
			return currentState;
		}
		switch (currentState) {

			case BALL_HOLDING:
				if (input.isLeftJoystickTriggerRaw()) {
					return FSMState.INTAKING;
				} else if (input.isShooterButtonPressed() && !doneShooting) {
					return FSMState.SHOOTING;
				}

				return FSMState.BALL_HOLDING;

			case INTAKING:
				if (input.isLeftJoystickTriggerRaw()) {
					return FSMState.INTAKING;
				} else {
					return FSMState.BALL_HOLDING;
				}

			case SHOOTING:
				if (!doneShooting) {
					return FSMState.SHOOTING;
				} else {
					doneShooting = false;
					return FSMState.BALL_HOLDING;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in INTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		setShooterPosition(1);

		if (input.isLeftJoystickTriggerRaw()) {
			setShooterPower(-0.1);
		}
	}

	/**
	 * Handle behavior in BALL_HOLDING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleBallHoldingState(TeleopInput input) {


		if (shooterDriver.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
			System.out.println("f" + shooterDriver.getForwardLimitSwitch(Type.kNormallyOpen));
			System.out.println("r" + shooterDriver.getReverseLimitSwitch(Type.kNormallyOpen));

			shooterDriver.set(-0.3);
		}


		// setShooterPower(0);

		// setShooterPosition(2);
	}

	/**
	 * Handle behavior in SHOOTING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleShootingState(TeleopInput input) {
		setShooterPosition(1);

		if (!doneShooting) {
			setShooterPower(0.6);
			if (shooterRight.getEncoder().getVelocity() >= 1000) {
				setShooterPosition(2);
			}
		}

		doneShooting = true;
	}

	/* ------------------------ Helpers ------------------------ */

	/**
	 * Sets the power for both shooting motors.
	 * @param pow The power at which the motors will be set to.
	 */
	private void setShooterPower(double pow) {
		shooterLeft.set(pow);
		shooterRight.set(pow);
	}

	/**
	 * Sets the position of the shooter.
	 * posValue = 1; Shooter will go up
	 * posValue = 2; Shooter will go down
	 * @param posValue The value for which setting the shooter should be at
	 */
	private void setShooterPosition(int posValue) {
		// Move shooter up
		if (posValue == 1) {


			if (!(magneticProxLeft.get() && magneticProxMid.get() && !magneticProxRight.get())) {
				shooterDriver.set(-0.1);
			}
		} else if (posValue == 2) { // Move shooter down
			if (!(!magneticProxLeft.get() && !magneticProxMid.get() && magneticProxRight.get())) {
				shooterDriver.set(0.1);
			}
		}
	}

}
