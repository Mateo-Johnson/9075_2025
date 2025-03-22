package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {

  public static CommandXboxController primary = new CommandXboxController(OIConstants.kDriverControllerPort);

  public static final class DriveConstants {
    // Be careful, these are the max allowed speeds, not the max capable
    public static final double kMaxSpeedMetersPerSecond = 5.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians/second

    // Chassis Config

    // Distance between the center of right and left modules 
    public static final double kTrackWidth = Units.inchesToMeters(23.75); // TUNE THIS, THIS SHOULD MATCH THE WITDH OF THE ROBOT
    // Distance between the center of the front and back modules
    public static final double kWheelBase = Units.inchesToMeters(23.75); // TUNE THIS, THIS SHOULD MATCH THE LENGTH OF THE ROBOT

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Rear Left
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Rear Right

    // SparkMax CAN IDs

    // In the email sent over, the first CANID was listed as 1? 1 is generally the RIO, so I am going to set it to 9
    // Driving
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 1; // THIS ONE
    public static final int kRearRightDrivingCanId = 3;

    // Turning
    public static final int kFrontLeftTurningCanId = 7;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 4;

    // Encoders
    public static final int kFrontLeftEncoder = 16;
    public static final int kFrontRightEncoder = 17;
    public static final int kRearLeftEncoder = 19;
    public static final int kRearRightEncoder = 18;

    // Gyro
    public static final int pigeonID = 20; 

    // Module offsets
    public static final double kFrontLeftOffset = -0.30029296875; // TUNE THIS
    public static final double kFrontRightOffset = -0.480224609375; // TUNE THIS
    public static final double kRearLeftOffset = -0.68310546875; // TUNE THIS
    public static final double kRearRightOffset = -0.77685546875; // TUNE THIS
  }

  public static final class ModuleConstants {
    
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // MK4i gear ratio
    public static final double kDrivingMotorReduction = 8.14; // L1 ratio
    public static final double kTurningMotorReduction = 21.4285714286; // MK4i ratio

    // Motor attributes
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) /
      kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // Controller port
    public static final double kDriveDeadband = 0.05;
  }


  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

}