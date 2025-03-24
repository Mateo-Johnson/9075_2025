// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coral extends SubsystemBase {
  private SparkMax coralMotor = new SparkMax(9, null);
  /** Creates a new coral. */
  public coral() {
    coralMotor.configFactoryDefault():
    coralMotor.setInverted(false);
    coralMotor.configPeakCurrentlimit(amps:20);
  }
   
  public void setSpeed(double speed){
    this.speed = speed;
    coralMotor.set(ControlMode.PercentOutput, speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
