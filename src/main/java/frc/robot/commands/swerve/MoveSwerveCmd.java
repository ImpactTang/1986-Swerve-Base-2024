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

public class MoveSwerveCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private double xSpeed, ySpeed, turningSpeed;
    private final SlewRateLimiter xSpdLimiter, ySpdLimiter, turningSpdLimiter;

    /* --------------------> Swerve Joystick Command Constructor <-------------------- */
    public MoveSwerveCmd(SwerveSubsystem swerveSubsystem, double xSpeed, double ySpeed, double turningSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.turningSpeed = turningSpeed;
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

        /* --------------------> Limit Speed Rate of Change <-------------------- */
        xSpeed = xSpdLimiter.calculate(xSpeed) * DriveConstants.kDriveMaxSpeedMetersPerSecond;
        ySpeed = ySpdLimiter.calculate(ySpeed) * DriveConstants.kDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningSpdLimiter.calculate(turningSpeed) * DriveConstants.kDriveMaxAngularSpeedRadiansPerSecond; // Limit Turning Rate of Change, then multiply by Max Turning Speed

        /* --------------------> Put Input Speeds on SmartDashboard <-------------------- */
        SmartDashboard.putNumber("Input X Speed", xSpeed);
        SmartDashboard.putNumber("Input Y Speed", ySpeed);
        SmartDashboard.putNumber("Input Turning Speed", turningSpeed);

        /* --------------------> Field Oriented Logic <-------------------- */
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);

        /* --------------------> Set Module States <-------------------- */
        SwerveModuleState[] moduleStates = swerveSubsystem.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        /* --------------------> Put Module States & FOC on SmartDashboard <-------------------- */
        SmartDashboard.putString("Module States", moduleStates.toString());
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
