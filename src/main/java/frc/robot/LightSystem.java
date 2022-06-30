package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.DigitalOutput;

// Third party Hardware Imports

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class LightSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		lightOn, lightOff;
		
	}


	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private DigitalOutput lightRelay;

	/* ======================== Constructor ======================== */
	/**
	 * Create LightSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public LightSystem() {
		// Perform hardware init
		lightRelay = new DigitalOutput(HardwareMap.LIGHT_RELAY_CHANNEL); // TODO: Instantiate lightRelay
	
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
		currentState = FSMState.lightOff;// TODO: Define start state

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
			// TODO: Assign state handlers for each FSM state
			case lightOn:
				handleOn(input);
				break;

			case lightOff:
			handleOff(input);
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
		// Do not run in Autonomous
		if (input == null) {
			return currentState;
		}
		// TODO: Define state transitions
		switch (currentState) {
			case lightOn:
				if (input.isOnButtonPressed()) {
					return FSMState.lightOn;
				} else {
					return FSMState.lightOff;
				}

			case lightOff:
				if(input.isOnButtonPressed()) {
					return FSMState.lightOn;
				} else {
					return FSMState.lightOff;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in ABC_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleOn (TeleopInput input) {
		lightRelay.set(true);
	}

	private void handleOff (TeleopInput input) {
		lightRelay.set(false);
	}
}
