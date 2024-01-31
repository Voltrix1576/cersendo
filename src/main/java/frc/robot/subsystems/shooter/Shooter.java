// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Shooter instance;
  private final CANSparkFlex masterShooting = new CANSparkFlex(0, MotorType.kBrushless);
  private final CANSparkFlex slaveShooting = new CANSparkFlex(0, MotorType.kBrushless);
  private final CANSparkMax masterArm = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax slaveArm = new CANSparkMax(0, MotorType.kBrushless);
  
  public Shooter() {
    slaveShooting.follow(masterShooting, true);
    slaveArm.follow(masterArm, false);

  }

  public void shootingSetPower(double power) {
    masterShooting.set(power);
  }

  public void AdjustSetPower(double power) {
    masterArm.set(power);
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
