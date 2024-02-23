// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.net.Socket;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.swerveDriveCommand;
// import frc.robot.commands.Autos.autoLine;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static final CommandPS4Controller driverController = new CommandPS4Controller(0); 
  public static final CommandXboxController operatorController = new CommandXboxController(1);
  private double startTime;
  // public final SwerveDrivetrain swerve = new SwerveDrivetrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   driverController.triangle().whileTrue(
    new InstantCommand(() -> SwerveDrivetrain.getInstance().updateOffset()));
    
   driverController.R1().whileTrue(
    new InstantCommand(() -> SwerveDrivetrain.getInstance().setVelocityFactor(0.5)))
    .whileFalse(new InstantCommand(() -> SwerveDrivetrain.getInstance().setVelocityFactor(1))); 
    
    driverController.R2().whileTrue(
    new InstantCommand(() -> SwerveDrivetrain.getInstance().setVelocityFactor(0.25)))
    .whileFalse(new InstantCommand(() -> SwerveDrivetrain.getInstance().setVelocityFactor(1))); 

    operatorController.a().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> Shooter.getInstance().shootAmp(0.1)),
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        new WaitUntilCommand(() -> Timer.getFPGATimestamp() - startTime >= 2), //delay to start conveyr
        new InstantCommand(() -> Shooter.getInstance().startConveyer(0.15)),
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        new WaitUntilCommand(() -> Timer.getFPGATimestamp() - startTime >= 1),
        new InstantCommand(() -> Shooter.getInstance().resetShoot())
      )
    );

    operatorController.rightBumper().onTrue(
      new SequentialCommandGroup(new InstantCommand(() -> Shooter.getInstance().shootSpeaker(0.6)),
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        new WaitUntilCommand(() -> Timer.getFPGATimestamp() - startTime >= 1.5), //delay to start conveyr
        new InstantCommand(() -> Shooter.getInstance().startConveyer(0.6)),
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        new WaitUntilCommand(() -> Timer.getFPGATimestamp() - startTime >= 1),
        new InstantCommand(() -> Shooter.getInstance().resetShoot())
        )
    );

    operatorController.leftBumper().onTrue(
      new InstantCommand(() -> Shooter.getInstance().resetArm())
    );

    operatorController.b().onTrue(
    new InstantCommand(() -> Shooter.getInstance().shootInTake(-0.15)))
    .onFalse(new InstantCommand(() -> Shooter.getInstance().shootInTake(0)));
    
    operatorController.x().whileTrue(
    new InstantCommand(() -> Shooter.getInstance().AdjustSetPower(0.1)))
    .whileFalse(new InstantCommand(() -> Shooter.getInstance().setAdjustVolt(0.38)));
  
   operatorController.y().whileTrue(
    new InstantCommand(() -> Shooter.getInstance().AdjustSetPower(-0.2)))
    .whileFalse(new InstantCommand(() -> Shooter.getInstance().setAdjustVolt(0.38)));

  //  operatorController.rightTrigger().whileTrue(
  //   new InstantCommand(() -> Shooter.getInstance().shootAmp(0.15)))
  //   .whileFalse(new InstantCommand(() -> Shooter.getInstance().setAdjustVolt(0.38))); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
