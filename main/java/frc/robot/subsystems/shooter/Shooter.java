// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Shooter extends SubsystemBase {
  private static Shooter instance;

  private final CANSparkFlex frontMasterShooting = new CANSparkFlex(
    ShooterConstance.frontMasterShootingID, MotorType.kBrushless);
 
    private final CANSparkFlex frontSlaveShooting = new CANSparkFlex(
    ShooterConstance.frontSlaveShootingID, MotorType.kBrushless);
 
    private final CANSparkMax masterArm = new CANSparkMax(
    ShooterConstance.masterArmID, MotorType.kBrushless);
 
    private RelativeEncoder armEncoder = masterArm.getEncoder();

    private final CANSparkMax slaveArm = new CANSparkMax(
    ShooterConstance.slaveArmID, MotorType.kBrushless);
  
    private final CANSparkFlex backMasterShooter = new CANSparkFlex(
    ShooterConstance.backMasterShootingID, MotorType.kBrushed);

    private final TalonSRX backSlaveShooter = new TalonSRX(ShooterConstance.backSlaveShootingID);

   // private final Servo armServo = new Servo(0);

    private final SparkPIDController pidControllerAdjust;

    
 

  public Shooter() {
    frontSlaveShooting.follow(frontMasterShooting, true);
    slaveArm.follow(masterArm);
    pidControllerAdjust = masterArm.getPIDController();
    pidControllerAdjust.setP(ShooterConstance.adjustkp);
    pidControllerAdjust.setI(ShooterConstance.adjustki);
    pidControllerAdjust.setD(ShooterConstance.adjustkd);
    pidControllerAdjust.setOutputRange(-0.2, 0.2);
  }

  public void shootOutTake(double power) {
    
    frontMasterShooting.set(power);
    backMasterShooter.set(power);
    backSlaveShooter.set(TalonSRXControlMode.PercentOutput, power);
    
  }

  public void shootAmp(double power) {
    calculate(11.96);
    frontMasterShooting.set(power);
  }

  public void shootSpeaker(double power) {
       calculate(13.69);
       frontMasterShooting.set(power);
  }

  public void startConveyer(double power) {
    backMasterShooter.set(power);
    backSlaveShooter.set(TalonSRXControlMode.PercentOutput, power);
  }

  public void resetArm()
  {
      calculate(2.98);
  }

  public void resetShoot(){
    backMasterShooter.set(0);
    backSlaveShooter.set(TalonSRXControlMode.PercentOutput, 0);
    frontMasterShooting.set(0);
  }

   public void shootInTake(double power) {
    calculate(13.69);
    frontMasterShooting.set(power);
    backMasterShooter.set(power);
    backSlaveShooter.set(TalonSRXControlMode.PercentOutput, power);
  }

  public void stop() {
    shootAmp(0);
    shootInTake(0);
  }

  public void AdjustSetPower(double power) {
    masterArm.set(power);
  }  

  public void calculate(double setPoint) {
    pidControllerAdjust.setReference(setPoint, ControlType.kPosition);
}
  
  public void setAdjustVolt(double voltage)
  {
      masterArm.setVoltage(voltage);
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("POSE", armEncoder.getPosition());
    //calculate(22);
  }
}
