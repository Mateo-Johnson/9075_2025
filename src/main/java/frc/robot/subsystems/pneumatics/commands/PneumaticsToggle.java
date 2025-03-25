package frc.robot.subsystems.pneumatics.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pneumatics.Pneumatics;

public class PneumaticsToggle extends Command {

  private final Pneumatics m_pneumatics;
  private boolean hasToggled = false;

  /** Creates a new PneumaticsToggle. */
  public PneumaticsToggle(Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pneumatics = pneumatics;
    addRequirements(m_pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasToggled = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasToggled) {
      m_pneumatics.pneumaticsOut(); // First call goes out
      hasToggled = true;
    } else {
      m_pneumatics.pneumaticsIn(); // Second call goes in
      end(false); // End the command after toggling
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Optional: Add any cleanup logic if needed
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasToggled; // Will end after second execute call
  }
}