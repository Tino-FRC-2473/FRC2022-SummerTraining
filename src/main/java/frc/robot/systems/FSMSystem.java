package frc.robot.systems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.HardwareMap;
import frc.robot.TeleopInput;

public class FSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_PHASE,
		PHASE_2,
		PHASE_3,
		PHASE_4
	}

	private static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private DoubleSolenoid sol;
	private CANSparkMax motor;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		sol = new DoubleSolenoid(PneumaticsModuleType.REVPH, HardwareMap.PCM_CHANNEL_INTAKE_CYLINDER_FORWARD, HardwareMap.PCM_CHANNEL_INTAKE_CYLINDER_REVERSE);
		motor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_LEFT, CANSparkMax.MotorType.kBrushless);
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
		currentState = FSMState.START_PHASE;
		update(null);
	}

	public void update(TeleopInput input) {
		switch (currentState) {
			case START_PHASE:
				handleStartPhase(input);
				break;
			case PHASE_2:
				handlePhase2(input);
				break;
			case PHASE_3:
				handlePhase3(input);
				break;
			case PHASE_4:
				handlePhase4(input);
				break;
		}
		currentState = nextState(input);
	}

	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case START_PHASE:
				return FSMState.PHASE_2;
			case PHASE_2:
				return FSMState.PHASE_3;
			case PHASE_3:
				return FSMState.PHASE_4;
			case PHASE_4:
				return FSMState.PHASE_2;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	private void handleStartPhase(TeleopInput input) {
		do {
			sol.set(Value.kForward);
		} while (!(sol.get() == DoubleSolenoid.Value.kOff));
	}

	private void handlePhase2(TeleopInput input) {
		double start_pos = motor.get();
		double placeholder = 69.0;
		while (motor.get() - start_pos < placeholder) {
			motor.set(MOTOR_RUN_POWER);
		}
		motor.set(0);
	}

	private void handlePhase3(TeleopInput input) {
		int ticks = 100;
		do {
			sol.set(Value.kReverse);
		} while (!(sol.get() == DoubleSolenoid.Value.kOff));
		
		double start = motor.get();
		while (start - motor.get() > ticks) {
			motor.set(-MOTOR_RUN_POWER);
		}
	}

	private void handlePhase4(TeleopInput input) {
		do {
			sol.set(Value.kForward);
		} while (!(sol.get() == DoubleSolenoid.Value.kOff));
	}
}
