package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax TurningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder TurningEncoder;

    private final SparkPIDController pidControllerDrive;
    private final SparkPIDController pidControllerTurning;

    private final SimpleMotorFeedforward feedforward;

    private final CANcoder absulotEncoder;

    private final double absulotEncoderOffset;

    private final boolean isAbsulotEncoderInverted;

    public SwerveModule(int driveMotorId, int TurningMotorId, int absulotEncoderid,  
     double absulotEncoderOffset, boolean isDriveMotorInverted, 
        boolean isTurningMotorInverted, boolean isAbsulotEncoderInverted) {
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        TurningMotor = new CANSparkMax(TurningMotorId, MotorType.kBrushless);
        absulotEncoder = new CANcoder(absulotEncoderid);

        TurningMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.setInverted(isDriveMotorInverted);
        TurningMotor.setInverted(isTurningMotorInverted);

        driveEncoder = driveMotor.getEncoder();
        TurningEncoder = TurningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(SwerveConstants.poseDriveFactor);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.velDriveFactor);
        TurningEncoder.setPositionConversionFactor(SwerveConstants.poseTurningFactor);


        pidControllerDrive = driveMotor.getPIDController();
        pidControllerTurning = TurningMotor.getPIDController();
    
        pidControllerDrive.setP(SwerveConstants.drivKp);
        pidControllerDrive.setI(SwerveConstants.drivKi);
        pidControllerDrive.setD(SwerveConstants.drivKd);

        pidControllerTurning.setP(SwerveConstants.turningKp);
        pidControllerTurning.setI(SwerveConstants.turningKi);
        pidControllerTurning.setD(SwerveConstants.turningKd);

        feedforward = new SimpleMotorFeedforward(SwerveConstants.KSDrive, SwerveConstants.KVDrive);
       
        this.absulotEncoderOffset = absulotEncoderOffset;
        this.isAbsulotEncoderInverted = isAbsulotEncoderInverted;

        resetEncoders();
    }
    
    public double getTurningPose() {
        return TurningEncoder.getPosition();
    }

    public double getDrivePose() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningPose()));
    }

    public double getAbsulotEncoderPose() {
       StatusSignal <Double> pose = absulotEncoder.getAbsolutePosition();
       pose.refresh();
       // return pose.getValue();
        return isAbsulotEncoderInverted ? 360 - ((pose.getValue() + 0.5)) * 360
         : ((pose.getValue() + 0.5)) * 360;
    }

    

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        TurningEncoder.setPosition(getAbsulotEncoderPose() - absulotEncoderOffset);
    }

    public void brakeMotors() {
        driveMotor.setIdleMode(IdleMode.kBrake);
        TurningMotor.setIdleMode(IdleMode.kBrake);
    }

    public void driveMotorSetPower(double power) {
        driveMotor.set(power);
    }

    public void TurningMotorSetPower(double power) {
        TurningMotor.set(power);
    }    

    public void stop() {
        driveMotorSetPower(0);
        TurningMotorSetPower(0);
    }

    public void driveUsingPID(double setPoint) {
        pidControllerDrive.setReference(
            setPoint, ControlType.kVelocity, 0, 
            feedforward.calculate(setPoint), ArbFFUnits.kPercentOut);
    }

    public void turningUsingPID(double setPoint) {
        pidControllerTurning.setReference(setPoint, ControlType.kPosition);
    }

    public SwerveModulePosition getModulePose() {
        return new SwerveModulePosition(
            getDrivePose(), Rotation2d.fromDegrees(getTurningPose()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModule.optimize(desiredState,
        getTurningPose());
        if (optimizedState.speedMetersPerSecond != 0) {
            turningUsingPID(optimizedState.angle.getDegrees());
        } else {
            TurningMotorSetPower(0);
        }
        driveUsingPID(optimizedState.speedMetersPerSecond);
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState,
        double currentAngle) {
        double angleDiff = (desiredState.angle.getDegrees() - currentAngle) % 360;
        double targetAngle = currentAngle  + angleDiff;
        double targetSpeed = desiredState.speedMetersPerSecond;  
    
        if (angleDiff <= -270) {
            targetAngle += 360;
        } else if (-90 > angleDiff && angleDiff > -270) {
            targetAngle += 180;
            targetSpeed = -targetSpeed;
        } else if (90 < angleDiff && angleDiff < 270) {
            targetAngle -= 180;
            targetSpeed = -targetSpeed;
        } else if (angleDiff >=  270) {
            targetAngle -= 360;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));

    }
}


