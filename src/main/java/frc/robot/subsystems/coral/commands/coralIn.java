// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class coralIn extends Command {
  private final coral coral;
  private final double speed; 

  /** Creates a new coralUp. */
  public coralIn(coral coral, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coral = coral; 
    this.speed = -Math.abs(speed);
    addRequirements(coral);
  }

  public coralIn(coral coral){
    this(coral, 0.4); // UPDATE SPEED uytrtyuioiuytrewdfgytrdcvbnmkiuytr
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.setSpeed(0); // UPDATE SPEED tresx m,lujdfamkrfoiaeijriadjagskfauihfahii
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
