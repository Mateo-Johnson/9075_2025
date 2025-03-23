package frc.robot.subsystems.elevator;

// import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Ignore all of the things that are commented out, they are not relevant to the current lift system - Mateo
public class Elevator extends SubsystemBase {
    
    private final SparkMax motor = new SparkMax(11, MotorType.kBrushless);
    // private final SparkClosedLoopController pidController = motor.getClosedLoopController();
    // private final RelativeEncoder encoder = motor.getEncoder();

    // private static final double kP = 0.1;  // Tune these values
    // private static final double kI = 0.0;
    // private static final double kD = 0.0;
    // private static final double kF = 0.0;

    // private double targetPosition = 0.0;

    public Elevator() {
        // Configure motor
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true)  // Adjust based on your robot's configuration
            .idleMode(IdleMode.kBrake);
        
        // Configure closed loop
        // config.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     .pid(kP, kI, kD);
        
        //Apply the motor config
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // // Ensure encoder position starts at 0
        // encoder.setPosition(0);
    }

    /**
     * Sets the elevator position in rotations.
     * @param rotations The target position in rotations
     */
    // public void setPosition(double rotations) {
    //     targetPosition = rotations;
    //     pidController.setReference(rotations, ControlType.kPosition);
    // }

    /**
     * Gets the current position in rotations.
     * @return Current position in rotations
     */
    // public double getPosition() {
    //     return encoder.getPosition();
    // }

    /**
     * Manually control the elevator with direct motor power.
     * @param speed Speed from -1.0 to 1.0
     */
    public void manualControl(double speed) {
        motor.set(speed);
    }

    /**
     * Reset the encoder position to zero.
     */
    // public void resetEncoder() {
    //     encoder.setPosition(0);
    // }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Elevator Position (Rotations)", encoder.getPosition());
        // SmartDashboard.putNumber("Elevator Target (Rotations)", targetPosition);
    }
}