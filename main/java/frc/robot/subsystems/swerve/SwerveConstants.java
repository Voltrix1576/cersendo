package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveConstants {
    private static final double gearDrive = 6.75;
    private static final double gearTurning = 150d / 7d;
    private static final double weelRaduis = 0.0508;

    public static final double poseDriveFactor = (weelRaduis * Math.PI * 2) / gearDrive;
    public static final double poseTurningFactor = (360 / gearTurning) * 1.636;

    public static final double velDriveFactor = poseDriveFactor / 60;

    public static final double length = 0.56; 
    public static final double width = 0.57; 

    
    // drive pid
    public static final double KVDrive = 0.21;
    public static final double KSDrive = 0;
    public static final double drivKp = 0.06;
    public static final double drivKi = 0;
    public static final double drivKd = 0;

    // turning pid
    public static final double turningKp = 0.02;
    public static final double turningKi = 0;
    public static final double turningKd = 0;

    // rear left
    public static final int rearLeftDriveID = 14; 
    public static final int rearLeftTurningID = 13; 
    public static final int rearLeftAbsulotEncoderID = 4; 
    public static final double rearLeftAbsulotEncoderOffset = 51.94;
    public static final boolean rearLeftIstDriveMotorInverted  = true;  
    public static final boolean rearLeftIstTurningMotorInverted = true; 
    public static final boolean rearLeftIsAbsulotEncoderInverted = true; 

    // front left
    public static final int frontLeftDriveID = 15; 
    public static final int frontLeftTurningID = 8; 
    public static final int frontLeftAbsulotEncoderID = 6; 
    public static final double frontLeftAbsulotEncoderOffset = 174.90; 
    public static final boolean frontLeftIstDriveMotorInverted = true;  
    public static final boolean frontLeftIstTurningMotorInverted = true;
    public static final boolean frontLeftIsAbsulotEncoderInverted = true;

    // rear right
    public static final int rearRightDriveID = 12; 
    public static final int rearRightTurningID = 9; 
    public static final int rearRightAbsulotEncoderID = 2; 
    public static final double rearRightAbsulotEncoderOffset =  266.66; 
    public static final boolean rearRightIstDriveMotorInverted = true; 
    public static final boolean rearRightIstTurningMotorInverted = true;
    public static final boolean rearRightIsAbsulotEncoderInverted = true;

    // front right 
    public static final int frontRightDriveID = 7; 
    public static final int frontRightTurningID = 11; 
    public static final int frontRightAbsulotEncoderID = 5; 
    public static final double frontRightAbsulotEncoderOffset = 89.38; 
    public static final boolean frontRightIstDriveMotorInverted = true; 
    public static final boolean frontRightIstTurningMotorInverted = true; 
    public static final boolean frontRightIsAbsulotEncoderInverted = true;

    private static final double robotRaduis = Math.sqrt(Math.pow(length, 2) + Math.pow(width, 2)) / 2d;

    public static final double maxV = 4.60248;
    public static final double maxAV = maxV / robotRaduis;

    public static final double maxAcceleration = 3; //TODO
    public static final double maxAngolarAcceleration = maxAcceleration / robotRaduis;

    public static final PIDController xController = new PIDController(0.1, 0, 0);
    public static final PIDController yController = new PIDController(0.1, 0, 0);
    public static final ProfiledPIDController angleController = 
    new ProfiledPIDController(0.1, 0, 0,
        new TrapezoidProfile.Constraints(maxAV, maxAngolarAcceleration)); 
        //ask kaplan if i should change the 0.1 to the new p after i changed
}
