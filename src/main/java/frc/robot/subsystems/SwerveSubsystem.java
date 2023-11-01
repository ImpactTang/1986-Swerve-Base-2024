package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants.*;

public class SwerveSubsystem extends SubsystemBase {

    /* --------------------> Swerve Modules <-------------------- */
    private final SwerveModule frontLeft = new SwerveModule(
        FrontLeftModule.driveMotorId, FrontLeftModule.turnMotorId, FrontLeftModule.driveMotorDirection, 
        FrontLeftModule.turnCanCoderId, FrontLeftModule.turnCanCoderDirection, FrontLeftModule.turnCanCoderOffset, "Front Left");
    private final SwerveModule frontRight = new SwerveModule(
        FrontRightModule.driveMotorId, FrontRightModule.turnMotorId, FrontRightModule.driveMotorDirection, 
        FrontRightModule.turnCanCoderId, FrontRightModule.turnCanCoderDirection, FrontRightModule.turnCanCoderOffset, "Front Left");
    private final SwerveModule backLeft = new SwerveModule(
        BackLeftModule.driveMotorId, BackLeftModule.turnMotorId, BackLeftModule.driveMotorDirection, 
        BackLeftModule.turnCanCoderId, BackLeftModule.turnCanCoderDirection, BackLeftModule.turnCanCoderOffset, "Front Left");
    private final SwerveModule backRight = new SwerveModule(
        BackRightModule.driveMotorId, BackRightModule.turnMotorId, BackRightModule.driveMotorDirection, 
        BackRightModule.turnCanCoderId, BackRightModule.turnCanCoderDirection, BackRightModule.turnCanCoderOffset, "Front Left");

    private final Pigeon2 gyro;
    
    public SwerveDriveOdometry swerveDriveOdometry;

    public SwerveSubsystem() {

        /* --------------------> Swerve Drive Constructor <-------------------- */
        gyro = new Pigeon2(50, "Canivore");
        configGyro(gyro);
        gyro.setYaw(0);

        swerveDriveOdometry = new SwerveDriveOdometry(DriveConstants.kSwerveDriveKinematics, getRotation2d(), getModulePositions());

        resetModuleEncoders();

    }

    /* --------------------> Periodic Updates <-------------------- */
    @Override
    public void periodic() {
        swerveDriveOdometry.update(getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    /* --------------------> Set Module States <-------------------- */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /* --------------------> Get Module Positions <-------------------- */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    /* --------------------> Set Robot Heading <-------------------- */
    public void setHeading(double heading){
        gyro.setYaw(heading); // Mainly Used to Reset Field Oriented Control
    }

    /* --------------------> Get Robot Rotation2d <-------------------- */
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    /* --------------------> Get Robot Pose <-------------------- */
    public Pose2d getPose() {
        return swerveDriveOdometry.getPoseMeters();
    }

    /* --------------------> Stop Swerve Modules <-------------------- */
    public void stopSwerve(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /* --------------------> Resetting Module Motor Encoders <-------------------- */
    public void resetModuleEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    /* --------------------> Gyro Configuration <-------------------- */
    private void configGyro(Pigeon2 gyro){
        Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
        gyroConfig.MountPose.MountPosePitch = 0;
        gyroConfig.MountPose.MountPoseRoll = 0;
        gyroConfig.MountPose.MountPoseYaw = 0;

        Pigeon2Configurator pigeon2Configurator = gyro.getConfigurator();
        pigeon2Configurator.apply(gyroConfig, 5);
    }
}
