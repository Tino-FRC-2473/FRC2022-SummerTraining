package frc.robot.systems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.kauailabs.navx.frc.AHRS;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleOpSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		OTHER_STATE, TELEOP_STATE
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax exampleMotor;
	private CANSparkMax rightMotor;
	private AnalogInput potentiometer = new AnalogInput(2);
	private AHRS gyro = new AHRS();
	private DigitalInput limitSwitch = new DigitalInput(5);
	private AnalogInput distanceSensor = new AnalogInput(3);

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public TeleOpSystem() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.MotorCANSparkID,
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
		currentState = FSMState.TELEOP_STATE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	protected void execute() {
		SmartDashboard.putNumber("Encoder Ticks", rightMotor.getEncoder().getPosition()); 
		SmartDashboard.putNumber("Potentiometer Voltage", potentiometer.getVoltage()); 
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle()); 
		SmartDashboard.putBoolean("Switches", limitSwitch.isAnalogTrigger()); 

		SmartDashboard.putNumber("Distance", distanceSensor.getVoltage()); 
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case START_STATE:
				handleStartState(input);
				break;

			case OTHER_STATE:
				handleOtherState(input);
				break;

			case TELEOP_STATE:
				handleTeleOpState(input);
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
				if (input != null) {
					return FSMState.OTHER_STATE;
				} else {
					return FSMState.START_STATE;
				}

			case OTHER_STATE:
				return FSMState.OTHER_STATE;
			case TELEOP_STATE:
				return FSMState.TELEOP_STATE;

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
		exampleMotor.set(0);
	}
	/**
	 * Handle behavior in OTHER_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOtherState(TeleopInput input) {
		exampleMotor.set(MOTOR_RUN_POWER);
	}

	private void handleTeleOpState(TeleopInput input) {
		if (input == null) {
			return; 
		} 

		execute();
	}
}