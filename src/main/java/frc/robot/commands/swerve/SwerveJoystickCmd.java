package frc.robot.commands.swerve;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdInput, ySpdInput, turningSpdInput;
    private final Supplier<Boolean> fieldOrientedInput;
    private final SlewRateLimiter xSpdLimiter, ySpdLimiter, turningSpdLimiter;

    /* --------------------> Swerve Joystick Command Constructor <-------------------- */
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdInput, Supplier<Double> ySpdInput, Supplier<Double> turningSpdInput, Supplier<Boolean> fieldOrientedInput) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdInput = xSpdInput;
        this.ySpdInput = ySpdInput;
        this.turningSpdInput = turningSpdInput;
        this.fieldOrientedInput = fieldOrientedInput;
        xSpdLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationRateUnitsPerSecond);
        ySpdLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationRateUnitsPerSecond);
        turningSpdLimiter = new SlewRateLimiter(DriveConstants.kMaxTurningRateUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    /* --------------------> Swerve Joystick Logic <-------------------- */
    @Override
    public void execute() {

        /* --------------------> Get Speed Command Values From Controller <-------------------- */
        double xSpeed = xSpdInput.get();
        double ySpeed = ySpdInput.get();
        double turningSpeed = turningSpdInput.get();

        /* --------------------> Deadband Logic <-------------------- */
        xSpeed = Math.abs(xSpeed) < IOConstants.kJoystickDeadband ? 0 : xSpeed;
        ySpeed = Math.abs(ySpeed) < IOConstants.kJoystickDeadband ? 0 : ySpeed;
        turningSpeed = Math.abs(turningSpeed) < IOConstants.kJoystickDeadband ? 0 : turningSpeed;

        /* --------------------> Limit Speed Rate of Change <-------------------- */
        xSpeed = xSpdLimiter.calculate(xSpeed) * DriveConstants.kDriveMaxSpeedMetersPerSecond;
        ySpeed = ySpdLimiter.calculate(ySpeed) * DriveConstants.kDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningSpdLimiter.calculate(turningSpeed) * DriveConstants.kDriveMaxAngularSpeedRadiansPerSecond; // Limit Turning Rate of Change, then multiply by Max Turning Speed

        /* --------------------> Put Input Speeds on SmartDashboard <-------------------- */
        SmartDashboard.putNumber("Input X Speed", xSpeed);
        SmartDashboard.putNumber("Input Y Speed", ySpeed);
        SmartDashboard.putNumber("Input Turning Speed", turningSpeed);

        ChassisSpeeds chassisSpeeds;

        /* --------------------> Field Oriented Logic <-------------------- */
        if (fieldOrientedInput.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d()); // Field Oriented if True
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed); // Robot Oriented if False
        }

        /* --------------------> Set Module States <-------------------- */
        SwerveModuleState[] moduleStates = DriveConstants.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        /* --------------------> Put Module States & FOC on SmartDashboard <-------------------- */
        SmartDashboard.putString("Module States", moduleStates.toString());
        SmartDashboard.putBoolean("Field Oriented", fieldOrientedInput.get());
    }

    /* --------------------> Stop Swerve Drive on Command End <-------------------- */
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopSwerve();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
