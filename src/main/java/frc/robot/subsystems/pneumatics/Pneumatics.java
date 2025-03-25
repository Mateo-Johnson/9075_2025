// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

  private final DoubleSolenoid actuators = 
    new DoubleSolenoid(PneumaticsModuleType.REVPH,0, 1);

  /** Creates a new Pneumatics. */

  //Extends the actuators
  public void pneumaticsOut() {
    actuators.set(DoubleSolenoid.Value.kForward);
  }

  //Retracts the actuators
  public void pneumaticsIn(){
    actuators.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
