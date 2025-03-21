package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.Config;

public class Module {
    private final SparkMax m_drivingSpark;
    private final SparkMax m_turningSpark;
    private final RelativeEncoder m_drivingEncoder;
    final RelativeEncoder m_turningEncoder;
    private final CANcoder m_turningAnalogEncoder;
    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;
    private final double m_encoderOffset;

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public Module(int drivingCANId, int turningCANId, int encoderPort, double analogOffset) {
        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getEncoder();
        m_turningAnalogEncoder = new CANcoder(encoderPort);
        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();
        m_encoderOffset = analogOffset;

        // Apply configurations
        m_drivingSpark.configure(Config.Module.drivingConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turningSpark.configure(Config.Module.turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
               
        // Reset the encoders during initialization
        syncAndZeroEncoders();
    }

    public double getAngle() { // I'm unsure about this, have never used cancoders.
        double rawAngle = m_turningAnalogEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        double adjustedAngle = rawAngle - m_encoderOffset;
        return Rotation2d.fromDegrees(adjustedAngle).getRadians();
    }

    private void syncAndZeroEncoders() {
      // Get the current absolute angle from the analog encoder
      double currentAngle = getAngle();
     
      // Set the turning encoder position to match the absolute encoder
      m_turningEncoder.setPosition(currentAngle);
     
      // Reset only the driving encoder
      m_drivingEncoder.setPosition(0);
     
      // Initialize the desired state to the current position
      m_desiredState = new SwerveModuleState(0.0, new Rotation2d(currentAngle));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                new Rotation2d(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_drivingEncoder.getPosition(),
                new Rotation2d(getAngle()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;

        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
       
        m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }


    public SwerveModuleState getDesiredState() {
      return m_desiredState;
    }

    public void resetWheels() {
        setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }

    public void resetEncoders() {
        syncAndZeroEncoders(); // This will reset the encoder values to zero
    }
}