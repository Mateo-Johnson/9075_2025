// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

//import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coral extends SubsystemBase {

  private final SparkMax coralMotor = new SparkMax(9, MotorType.kBrushless);
  /** Creates a new coral. */
  public coral() {
    
    // Configure motor 
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    //Apply the motor config
    //coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
   //Manually control the coral motor with direct motor power 
  public void setSpeed(double speed){
    coralMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
