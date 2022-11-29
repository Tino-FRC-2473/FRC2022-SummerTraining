package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.LimeLight;

public class CVFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		SHOOT_STATE,
		AIM_STATE,
		BALL_ALIGN,
		BALL_COLLECT
	}



	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor2;
	private CANSparkMax rightMotor2;
	private LimeLight limeLight;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public CVFSM() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT1,
										CANSparkMax.MotorType.kBrushless);
		rightMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT2,
										CANSparkMax.MotorType.kBrushless);
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT1,
										CANSparkMax.MotorType.kBrushless);
		leftMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT2,
										CANSparkMax.MotorType.kBrushless);
		limeLight = new LimeLight();
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
		currentState = FSMState.AIM_STATE;
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		limeLight.update();
		switch (currentState) {
			case SHOOT_STATE:
				handleShootState(input);
				break;
			case AIM_STATE:
				handleAimState(input);
				break;
			case BALL_COLLECT:
				//handleBallCollectState(input);
				break;
			case BALL_ALIGN:
				//handleBallAlignState(input);
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
			case SHOOT_STATE:
				if (limeLight.getHubTurningPower() ==0) {
					//if (finished shooting) {
						//return FSMState.BALL_ALIGN;
					//}else{
						return FSMState.SHOOT_STATE;
				} else {
					return FSMState.AIM_STATE;
				}

			case AIM_STATE:
				if (limeLight.getHubTurningPower() == 0) {
					return FSMState.SHOOT_STATE;
				}else{
					return FSMState.AIM_STATE;
				}

			case BALL_COLLECT:
				if (limeLight.getBallTurningPower() != 0) {
					return FSMState.BALL_ALIGN;
				}else{
					//if (finished intaking){
						//return FSMState.AIM_STATE;
					//}else{
						//return FSMState.BALL_COLLECT;
					//}
				}
			case BALL_ALIGN:
				//if (limeLight.getBallTurningPower() == 0) {
					//return FSMState.BALL_COLLECT;
				//}else{
					//return FSMState.BALL_ALIGN;
				//}
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
	private void handleAimState(TeleopInput input) {
		//moveCameraToHubLevel();
		if (limeLight.getHubTurningPower() != LimeLight.INVALID_RETURN) {
			rightMotor.set(0.1 * limeLight.getHubTurningPower());
			rightMotor2.set(0.1 * limeLight.getHubTurningPower());
			leftMotor.set(0.1 * limeLight.getHubTurningPower());
			leftMotor2.set(0.1 * limeLight.getHubTurningPower());
		} else {
			leftMotor.set(0.2);
			rightMotor.set(0.2);
			leftMotor2.set(0.2);
			rightMotor2.set(0.2);
		}
	}
	private void handleShootState(TeleopInput input) {
		//moveCameraToHubLevel();
		//double power = LimeLight.distanceToPower(limeLight.getHubDistance());
		//run shooter mech
	}
	/*
	private void handleBallAlignState(TeleopInput input) {
		//moveCameraToBallLevel();
		if (limeLight.getBallTurningPower() != LimeLight.INVALID_RETURN) {
			rightMotor.set(0.1 * limeLight.getBallTurningPower());
			rightMotor2.set(0.1 * limeLight.getBallTurningPower());
			leftMotor.set(0.1 * limeLight.getBallTurningPower());
			leftMotor2.set(0.1 * limeLight.getBallTurningPower());
		} else {
			leftMotor.set(0.2);
			rightMotor.set(0.2);
			leftMotor2.set(0.2);
			rightMotor2.set(0.2);
		}
	}

	private void handleBallCollectState(TeleopInput input) {
		//moveCameraToBallLevel();
		if (limeLight.getBallDistance() >= 0.2) {
			leftMotor.set(-0.3);
			rightMotor.set(0.3);
			leftMotor2.set(-0.3);
			rightMotor2.set(0.3);
		} else {
			leftMotor.set(-0.05);
			rightMotor.set(0.05);
			leftMotor2.set(-0.05);
			rightMotor2.set(0.05);
			//run intake mech
		}
	}
	*/
}