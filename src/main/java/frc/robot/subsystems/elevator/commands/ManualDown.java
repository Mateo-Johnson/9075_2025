package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ManualDown extends Command {
    private final Elevator elevator;
    private final double speed;
    
    /**
     * Creates a new ManualDown command.
     * 
     * @param elevator The elevator subsystem
     * @param speed Speed to move downward (positive value will be made negative)
     */
    public ManualDown(Elevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = -Math.abs(speed); // Ensure negative value
        addRequirements(elevator);
    }
    
    /**
     * Creates a new ManualDown command with default speed.
     * 
     * @param elevator The elevator subsystem
     */
    public ManualDown(Elevator elevator) {
        this(elevator, 0.5); // Default speed (will be made negative in constructor)
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
