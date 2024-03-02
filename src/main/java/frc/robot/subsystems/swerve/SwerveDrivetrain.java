// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {

  private static SwerveDrivetrain instance;

  public double maxV = SwerveConstants.maxV;
  public double maxAV = SwerveConstants.maxAV;

  public double offsetAngle = 0;

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  // private SwerveModulePosition[] getModulesPose() {
  //   return new SwerveModulePosition[] {
  //     frontLeftModule.getModulePose(),
  //     rearLeftModule.getModulePose(),
  //     frontRightModule.getModulePose(),
  //     rearRightModule.getModulePose()
  //   };
  // }

  // private final SwerveDriveOdometry odometry = 
  //   new SwerveDriveOdometry(getKinematics(), getRotation2d(), getModulesPose());

  private SwerveModule frontLeftModule = 
    new SwerveModule(
      SwerveConstants.frontLeftDriveID,
      SwerveConstants.frontLeftTurningID,
      SwerveConstants.frontLeftAbsulotEncoderID,
      SwerveConstants.frontLeftAbsulotEncoderOffset,
      SwerveConstants.frontLeftIstDriveMotorInverted,
      SwerveConstants.frontLeftIstTurningMotorInverted,
      SwerveConstants.frontLeftIsAbsulotEncoderInverted);

  private SwerveModule frontRightModule = 
    new SwerveModule(
      SwerveConstants.frontRightDriveID,
      SwerveConstants.frontRightTurningID,
      SwerveConstants.frontRightAbsulotEncoderID,
      SwerveConstants.frontRightAbsulotEncoderOffset,
      SwerveConstants.frontRightIstDriveMotorInverted,
      SwerveConstants.frontRightIstTurningMotorInverted,
      SwerveConstants.frontRightIsAbsulotEncoderInverted);

  private SwerveModule rearLeftModule = 
    new SwerveModule(
      SwerveConstants.rearLeftDriveID,
      SwerveConstants.rearLeftTurningID,
      SwerveConstants.rearLeftAbsulotEncoderID,
      SwerveConstants.rearLeftAbsulotEncoderOffset,
      SwerveConstants.rearLeftIstDriveMotorInverted,
      SwerveConstants.rearLeftIstTurningMotorInverted,
      SwerveConstants.rearLeftIsAbsulotEncoderInverted);  
      
  private SwerveModule rearRightModule = 
    new SwerveModule(
      SwerveConstants.rearRightDriveID,
      SwerveConstants.rearRightTurningID,
      SwerveConstants.rearRightAbsulotEncoderID,
      SwerveConstants.rearRightAbsulotEncoderOffset,
      SwerveConstants.rearRightIstDriveMotorInverted,
      SwerveConstants.rearRightIstTurningMotorInverted,
      SwerveConstants.rearRightIsAbsulotEncoderInverted);  
      
  private final Translation2d frontLeft = 
    new Translation2d(
      -SwerveConstants.width / 2,
      SwerveConstants.length / 2);    
 
  private final Translation2d rearLeft = 
    new Translation2d(
      -SwerveConstants.width / 2,
      -SwerveConstants.length / 2);

  private final Translation2d frontRight = 
    new Translation2d(
      SwerveConstants.width / 2,
      SwerveConstants.length / 2);
      
  private final Translation2d rearRight = 
    new Translation2d(
      SwerveConstants.width / 2,
      -SwerveConstants.length / 2); 

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeft, frontRight, rearLeft, rearRight);
    
  public SwerveDrivetrain() {
    navx.reset();
  }  
      
  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    rearLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  public void IdleModeMotors() {
    frontLeftModule.IdleModeMotors();
    rearLeftModule.IdleModeMotors();
    frontRightModule.IdleModeMotors();
    rearRightModule.IdleModeMotors();
  }

  public void setVelocityFactor(double factor) {
    maxV = SwerveConstants.maxV * factor;
    maxAV = SwerveConstants.maxAV * factor;
  }

  public double getAngle() {
    return navx.getYaw();
  }



  public void updateOffset() {
    offsetAngle = getAngle();
  }
      
  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getAngle()));
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void setSwerveState(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxV);
    frontLeftModule.setDesiredState(states[0]);
    rearLeftModule.setDesiredState(states[2]);
    frontRightModule.setDesiredState(states[1]);
    rearRightModule.setDesiredState(states[3]); 
  }

  public void drive(double xv, double yv, double omega, boolean isFieldrelative) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(
      isFieldrelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xv, yv, omega,
      new Rotation2d(Math.toRadians(getAngle() - offsetAngle)))
      : new ChassisSpeeds(xv, yv, omega)
    );
    setSwerveState(states);
  }

  // public Pose2d getPose() {
  //   return odometry.getPoseMeters();
  // }

  // public void resetOdometry(Pose2d pose) {
  //   odometry.resetPosition(getRotation2d(), getModulesPose(), pose);
  // }

  public static SwerveDrivetrain getInstance() {
    if (instance == null) {
      instance = new SwerveDrivetrain();
    }
    return instance;
  }
  
  
  

  @Override
  public void periodic() {
    // odometry.update(getRotation2d(), getModulesPose());

    // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    // drive(SwerveConstants.xController.calculate(getPose().getX(), 1), 0, 0, false);
 
    // drive(0, SwerveConstants.yController.calculate(getPose().getY(), 1), 0, false);

    // drive(0, 0, 
    //   SwerveConstants.angleController.calculate(getPose().getRotation().getRadians(),
    //  Math.toRadians(90)), false);

    // SmartDashboard.putString("pose", "(" + getPose().getX() + "," + getPose().getY() + ")");

    SmartDashboard.putNumber("fla", frontLeftModule.getAbsulotEncoderPose());
    SmartDashboard.putNumber("rla", rearLeftModule.getAbsulotEncoderPose());
    SmartDashboard.putNumber("fra", frontRightModule.getAbsulotEncoderPose());
    SmartDashboard.putNumber("rra", rearRightModule.getAbsulotEncoderPose());

    SmartDashboard.putNumber("fl angle", frontLeftModule.getTurningPose());
    SmartDashboard.putNumber("fr angle", frontRightModule.getTurningPose());
    SmartDashboard.putNumber("rl angle", rearLeftModule.getTurningPose());
    SmartDashboard.putNumber("rr angle", rearRightModule.getTurningPose());

    SmartDashboard.putNumber("navx", getAngle());    

  }
}
