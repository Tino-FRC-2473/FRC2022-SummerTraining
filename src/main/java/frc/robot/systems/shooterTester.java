package frc.robot.systems;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ShooterTester {
    public enum FSMState {
        IDLE,
        RUNNING,
        RETRACTING,
        INTAKE
    }

    private static final float intakePower = 0.2f;
    private static final float prepMotorPower = 0.65f;
    private static final float shooterPower = 0.95f;

    private FSMState currentState;

    private CANSparkMax interMotor1;
    private CANSparkMax interMotor2;
    private CANSparkMax prepMotor;
    private CANSparkMax shooterMotor;
    private CANSparkMax testMotor;

    public ShooterTester() {
        interMotor1 = new CANSparkMax(HardwareMap.TRANSFER_MOTOR_LEFT, CANSparkMax.MotorType.kBrushless);
        interMotor2 = new CANSparkMax(HardwareMap.TRANSFER_MOTOR_RIGHT, CANSparkMax.MotorType.kBrushless);
        prepMotor = new CANSparkMax(HardwareMap.INTER, CANSparkMax.MotorType.kBrushless);
        shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER, CANSparkMax.MotorType.kBrushed);

        testMotor = new CANSparkMax(13, CANSparkMax.MotorType.kBrushless);
    }

    public void reset() {
        currentState = FSMState.IDLE;
        // Call one tick of update to ensure outputs reflect start state
        // updateDashboard(null);
        update(null);
    }

    public void update(TeleopInput input) {
        switch (currentState) {
            case IDLE:
                interMotor1.set(0);
                interMotor2.set(0);
                prepMotor.set(0);
                shooterMotor.set(0);
                break;
            case RUNNING:
                // interMotor1.set(intakePower);
                // interMotor2.set(-intakePower);
                // prepMotor.set(prepMotorPower);
                // shooterMotor.set(shooterPower);

                testMotor.set(intakePower);
                break;
            case RETRACTING:
                // interMotor1.set(-intakePower);
                // interMotor2.set(intakePower);
                // prepMotor.set(-prepMotorPower);
                // shooterMotor.set(-shooterPower);
                break;
            case INTAKE:
                // interMotor1.set(-intakePower);
                // interMotor2.set(intakePower);
                // prepMotor.set(0);
                // shooterMotor.set(0);
                break;
            default:
                throw new IllegalStateException("Invalid state: " + currentState.toString());

        }

        currentState = nextState(input);
    }

    private FSMState nextState(TeleopInput input) {
        if (input == null) {
            return FSMState.IDLE;
        }
        if (input.isShooterButtonPressed()) {
            return FSMState.RUNNING;
        }
        if(input.isIntakeButtonPressed()) {
            return FSMState.INTAKE;
        }
        if (input.isEjectButtonPressed())
            return FSMState.RETRACTING; 
        
        return FSMState.IDLE;
    }
}
