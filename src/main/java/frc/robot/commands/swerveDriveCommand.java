// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class swerveDriveCommand extends Command {
  private SwerveDrivetrain swerve;

  public swerveDriveCommand() {
    swerve = SwerveDrivetrain.getInstance();
    // swerve.resetEncoders();    
    // swerve.IdleModeMotors();
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xv = RobotContainer.driverController.getLeftX() * swerve.maxV;
    double yv = RobotContainer.driverController.getLeftY() * swerve.maxV;
    double omega = RobotContainer.driverController.getRightX() * swerve.maxAV;

    if (Math.abs(RobotContainer.driverController.getLeftX()) < 0.1) {
      xv = 0;
    }
    if (Math.abs(RobotContainer.driverController.getLeftY()) < 0.1) {
      yv = 0;
    }
    if (Math.abs(RobotContainer.driverController.getRightX()) < 0.1) {
      omega = 0;
    }

    swerve.drive(xv, yv, -omega, true);
    // swerve.drive(1, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
