package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;


public class shooterTester {
    public enum FSMState {
		IDLE,
        RUNNING
	}

    private static final float intakePower = 0.1f;
    private static final float prepMotorPower = 0.3f;
    private static final float shooterPower = 0.8f;

    private FSMState currentState;

    private CANSparkMax interMotor1;
	private CANSparkMax interMotor2;
    private CANSparkMax prepMotor;
	private CANSparkMax shooterMotor;

    public shooterTester(){
        interMotor1 = new CANSparkMax(HardwareMap.INTER1, CANSparkMax.MotorType.kBrushless);
		interMotor2 = new CANSparkMax(HardwareMap.INTER2, CANSparkMax.MotorType.kBrushless);
        prepMotor = new CANSparkMax(HardwareMap.PREP, CANSparkMax.MotorType.kBrushless);
		shooterMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER, CANSparkMax.MotorType.kBrushed);
    }

    public void reset() {
		currentState = FSMState.IDLE;
		// Call one tick of update to ensure outputs reflect start state
		//updateDashboard(null);
		update(null);
	}

    public void update(TeleopInput input){
        switch(currentState){
            case IDLE:
                interMotor1.set(0);
                interMotor2.set(0);
                prepMotor.set(0);
                shooterMotor.set(0);
                break;
            case RUNNING:
                interMotor1.set(intakePower);
                interMotor2.set(-intakePower);
                prepMotor.set(prepMotorPower);
                shooterMotor.set(shooterPower);
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
		if(input.isIntakeButtonPressed()){
            return FSMState.RUNNING;
        }
        return FSMState.IDLE;
	}
}
