// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.coral;
import frc.robot.subsystems.coral.commands.coralIn;
import frc.robot.subsystems.coral.commands.coralOut;
//import frc.robot.subsystems.coral.coral;
//import frc.robot.subsystems.coral.commands.coralIn;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.ManualDown;
import frc.robot.subsystems.elevator.commands.ManualUp;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.lift.commands.LiftDown;
import frc.robot.subsystems.lift.commands.LiftUp;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.pneumatics.commands.PneumaticsIn;
import frc.robot.subsystems.pneumatics.commands.PneumaticsOut;
//import frc.robot.subsystems.pneumatics.commands.PneumaticsIn;
//import frc.robot.subsystems.pneumatics.commands.PneumaticsOut;
//import frc.robot.subsystems.pneumatics.commands.PneumaticsToggle;
import frc.robot.utils.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  final Drivetrain m_drivetrain;
  final Elevator m_elevator;
  final Pneumatics m_pneumatics;
  final Lift m_lift;
  final coral m_coral;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_componentController =
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    m_drivetrain = new Drivetrain();
    m_elevator = new Elevator();
    m_pneumatics = new Pneumatics();
    m_lift = new Lift();
    m_coral = new coral();

              m_drivetrain.setDefaultCommand( // IF THE DRIVETRAIN ISN'T DOING ANYTHING ELSE, DO THIS
        new RunCommand(() -> {
            m_drivetrain.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true);
        }, m_drivetrain)
    );

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_componentController.povUp().whileTrue(new ManualUp(m_elevator,0.8)); // Move elevator up when D-Pad up is pressed
    m_componentController.povDown().whileTrue(new ManualDown(m_elevator,0.1)); // Move elevator down when D-Pad down is pressed
    m_componentController.button(2).whileTrue(new PneumaticsIn(m_pneumatics)); //Move Pneumatics In
    m_componentController.button(3).whileTrue(new PneumaticsOut(m_pneumatics)); //Move Pneumatics Out 
    m_componentController.button(4).whileTrue(new LiftUp(m_lift, 1)); //Lift The Robot 
    m_componentController.button(1).whileTrue(new LiftDown(m_lift, 1)); //Lift The Robot 
    m_componentController.leftBumper().whileTrue(new coralIn(m_coral));
    m_componentController.rightBumper().whileTrue(new coralOut(m_coral));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}