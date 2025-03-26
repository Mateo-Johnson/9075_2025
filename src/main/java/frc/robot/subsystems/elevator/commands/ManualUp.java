package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ManualUp extends Command {
    private final Elevator elevator;
    private final double speed;
    
    /**
     * Creates a new ManualUp command.
     * 
     * @param elevator The elevator subsystem
     * @param speed Speed to move upward (positive value)
     */
    public ManualUp(Elevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = Math.abs(speed); // Ensure positive value
        addRequirements(elevator);
    }
    
    /**
     * Creates a new ManualUp command with default speed.
     * 
     * @param elevator The elevator subsystem
     */
    public ManualUp(Elevator elevator) {
        this(elevator, 0.7); // Default speed
    }
    
    @Override
    public void execute() {
        elevator.manualControl(speed);
    }
    
    @Override
    public void end(boolean interrupted) {
        elevator.manualControl(0);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}