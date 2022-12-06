package frc.robot.systems;
 
// WPILib Imports
 
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
 
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
 
public class HomemadePID {
    /* ======================== Constants ======================== */
    // FSM state definitions
    public enum FSMState {
        REST,
        SHOOT
    }
    private static final float MAX_SPEED = 0.15f;
    private static final float DESIRED_SPEED = 0.1f;
    private static final float P = 0.1f;
    /* ======================== Private variables ======================== */
    private FSMState currentState;
 
    // Hardware devices should be owned by one and only one system. They must
    // be private to their owner system and may not be used elsewhere.
    private CANSparkMax motor;
 
    /* ======================== Constructor ======================== */
    /**
     * Create FSMSystem and initialize to starting state. Also perform any
     * one-time initialization or configuration of hardware required. Note
     * the constructor is called only once when the robot boots.
     */
    public HomemadePID() {
        // Perform hardware init
        motor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
                CANSparkMax.MotorType.kBrushless);
        // Reset state machine
        reset();
    }
 
    /* ======================== Public methods ======================== */
    /**
     * Return current FSM state.
     *
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
        currentState = FSMState.REST;
 
        // Call one tick of update to ensure outputs reflect start state
        update(null);
    }
 
    /**
     * Update FSM based on new inputs. This function only calls the FSM state
     * specific handlers.
     *
     * @param input Global TeleopInput if robot in teleop mode or null if
     *              the robot is in autonomous mode.
     */
    public void update(TeleopInput input) {
        switch (currentState) {
            case REST:
                handleRestState(input);
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
     *
     * @param input Global TeleopInput if robot in teleop mode or null if
     *              the robot is in autonomous mode.
     * @return FSM state for the next iteration
     */
    private FSMState nextState(TeleopInput input) {
        if (input == null) {
            return FSMState.REST;
        }
        if (input.isShooterButtonPressed())
            return FSMState.SHOOT;
        return FSMState.REST;
    }
 
    /* ------------------------ FSM state handlers ------------------------ */
    /**
     * Handle behavior in START_STATE.
     *
     * @param input Global TeleopInput if robot in teleop mode or null if
     *              the robot is in autonomous mode.
     */
    private void handleRestState(TeleopInput input)
    {
        motor.set(0);
    }
    private double calc(double curSpeed)
    {
        return P * (DESIRED_SPEED - curSpeed);
    }
    private void handleShootState(TeleopInput input)
    {
        double velocity = calc(motor.getEncoder().getVelocity());
        velocity = Math.min(velocity, MAX_SPEED);
        velocity = Math.max(velocity, -MAX_SPEED);
        motor.set(velocity);
        System.out.println(motor.getEncoder().getVelocity());
    }
}

