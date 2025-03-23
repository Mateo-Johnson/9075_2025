// package frc.robot.subsystems.elevator;

// import edu.wpi.first.wpilibj2.command.Command;

// public class GoToPosition extends Command {
//     private final Elevator elevator;
//     private final double targetPosition;
    
//     /**
//      * Creates a new GoToPosition command.
//      * 
//      * @param elevator The elevator subsystem
//      * @param targetPosition The target position in rotations
//      */
//     public GoToPosition(Elevator elevator, double targetPosition) {
//         this.elevator = elevator;
//         this.targetPosition = targetPosition;
//         addRequirements(elevator);
//     }
    
//     @Override
//     public void initialize() {
//         elevator.setPosition(targetPosition);
//     }
    
//     @Override
//     public boolean isFinished() {
//         // Command finishes when position is within acceptable error
//         return Math.abs(elevator.getPosition() - targetPosition) < 0.1; // Adjust tolerance as needed
//     }
// }