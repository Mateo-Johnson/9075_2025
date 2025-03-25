// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

  private final SparkMax liftMotor = new SparkMax(11,MotorType.kBrushless);
  /** Creates a new Lift. */
  public Lift() {

    //Configure motor
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .inverted(false)
      .idleMode(IdleMode.kBrake);
  }

     //Manually control the coral motor with direct motor power 
     public void setSpeed(double speed){
      liftMotor.set(speed);
     }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
