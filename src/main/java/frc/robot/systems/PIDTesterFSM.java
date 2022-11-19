package frc.robot.systems;

import com.revrobotics.CANSparkMax;

// WPILib Imports

// Third party Hardware Imports
//import com.revrobotics.CANSparkMax;
// Robot Imports
import frc.robot.TeleopInput;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.HardwareMap;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDTesterFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		STOP_STATE,
		RUN_STATE
	}

	private RelativeEncoder shooterEncoder;
	private double kP;
	private double kI;
	private double kD;
	private double kIz;
	private double kFF;
	private double kMaxOutput;
	private double kMinOutput;
	private double maxRPM;
	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private CANSparkMax shooterMotor;
	private SparkMaxPIDController pid;
	private static final double MAX_OUTPUT = 0.3;
	private static final double MAX_RPM = 500;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public PIDTesterFSM() {
		shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
										CANSparkMax.MotorType.kBrushless);
		pid = shooterMotor.getPIDController();
		shooterEncoder = shooterMotor.getEncoder();

		// PID coefficients
		kP = 1;
		kI = 0;
		kD = 0;
		kIz = 0;
		kFF = 0;
		kMaxOutput = MAX_OUTPUT;
		kMinOutput = -MAX_OUTPUT;
		maxRPM = MAX_RPM;

		// set PID coefficients
		pid.setP(kP);
		pid.setI(kI);
		pid.setD(kD);
		pid.setIZone(kIz);
		pid.setFF(kFF);
		pid.setOutputRange(kMinOutput, kMaxOutput);

		// display PID coefficients on SmartDashboard
		SmartDashboard.putNumber("P Gain", kP);
		SmartDashboard.putNumber("I Gain", kI);
		SmartDashboard.putNumber("D Gain", kD);
		SmartDashboard.putNumber("I Zone", kIz);
		SmartDashboard.putNumber("Feed Forward", kFF);
		SmartDashboard.putNumber("Max Output", kMaxOutput);
		SmartDashboard.putNumber("Min Output", kMinOutput);
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
		currentState = FSMState.STOP_STATE;
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
			case STOP_STATE:
				handleStopState(input);
				break;
			case RUN_STATE:
				handleRunState(input);
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
			return FSMState.STOP_STATE;
		}
		if (input.isShooterButtonPressed()) {
			return FSMState.RUN_STATE;
		}
		return FSMState.STOP_STATE;
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStopState(TeleopInput input) {
		shooterMotor.set(0);
	}
	private void handleRunState(TeleopInput input) {
		double p = SmartDashboard.getNumber("P Gain", 0);
		double i = SmartDashboard.getNumber("I Gain", 0);
		double d = SmartDashboard.getNumber("D Gain", 0);
		double iz = SmartDashboard.getNumber("I Zone", 0);
		double ff = SmartDashboard.getNumber("Feed Forward", 0);
		double max = SmartDashboard.getNumber("Max Output", 0);
		double min = SmartDashboard.getNumber("Min Output", 0);

		// if PID coefficients on SmartDashboard have changed, write new values to controller
		if ((p != kP)) {
			pid.setP(p); kP = p;
		}
		if ((i != kI)) {
			pid.setI(i); kI = i;
		}
		if ((d != kD)) {
			pid.setD(d); kD = d;
		}
		if ((iz != kIz)) {
			pid.setIZone(iz); kIz = iz;
		}
		if ((ff != kFF)) {
			pid.setFF(ff); kFF = ff;
		}
		if ((max != kMaxOutput) || (min != kMinOutput)) {
			pid.setOutputRange(min, max);
			kMinOutput = min; kMaxOutput = max;
		}
		double setPoint = 500;
		pid.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
		SmartDashboard.putNumber("SetPoint", setPoint);
		SmartDashboard.putNumber("ProcessVariable", shooterEncoder.getVelocity());
	}
}
