package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

    private final int driveMotorId, turnMotorId, turnCanCoderId;
    private final double turnCanCoderOffset;
    private final String moduleName;

    private final TalonFX driveMotor, turnMotor;
    private final InvertedValue driveMotorDirection;
    private final PhoenixPIDController drivePidController, turningPidController;

    private final CANcoder turnCanCoder;
    private final SensorDirectionValue turnCanCoderDirection;

    /* --------------------> Swerve Module Constructor <-------------------- */
    public SwerveModule(int driveMotorId, int turnMotorId, InvertedValue driveMotorDirection, int turnCanCoderId, SensorDirectionValue turnCanCoderDirection, double turnCanCoderOffset, String moduleName) {

        this.driveMotorId = driveMotorId;
        this.turnMotorId = turnMotorId;
        this.driveMotorDirection = driveMotorDirection;
        this.turnCanCoderId = turnCanCoderId;
        this.turnCanCoderDirection = turnCanCoderDirection;
        this.turnCanCoderOffset = turnCanCoderOffset; // TODO: SET IN ROTATIONS
        this.moduleName = moduleName;

        /* --------------------> Drive Motor Config <-------------------- */
        driveMotor = new TalonFX(driveMotorId, "Canivore");
        configDriveMotor(driveMotor);

        /* --------------------> Turn Motor Config <-------------------- */
        turnMotor = new TalonFX(turnMotorId, "Canivore");
        configTurnMotor(turnMotor);

        /* --------------------> Turn CANCoder Config <-------------------- */
        turnCanCoder = new CANcoder(turnCanCoderId, "Canivore");
        configTurnCanCoder(turnCanCoder);

        /* --------------------> PID Controllers <-------------------- */
        drivePidController = new PhoenixPIDController(0.5, 0, 0);
        turningPidController = new PhoenixPIDController(0.5, 0, 0);
        
    }

    /* --------------------> Set Module States <-------------------- */
    public void setDesiredState(SwerveModuleState desiredState){
        
        // Remove unwanted module movement
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.1) {
            stop();
            return;
        }

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); // Optimize the desired state for smart pivot

        /* --------------------> Set Motor Speeds <-------------------- */
        driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.kDriveMaxSpeedMetersPerSecond);
        turnMotor.set(turningPidController.calculate(getTurningPositionRot(), desiredState.angle.getRotations(), 0.25) / DriveConstants.kDriveMaxAngularSpeedRadiansPerSecond);
        SmartDashboard.putNumber("Drive Motor Value", driveMotor.getPosition().getValueAsDouble()); // Update SmartDashboard with Drive Motor Value
        SmartDashboard.putNumber("Turn Motor Value", turnMotor.getPosition().getValueAsDouble()); // Update SmartDashboard with Turn Motor Value
        SmartDashboard.putString("Swerve["+moduleName+"] state", desiredState.toString()); // Update SmartDashboard with Module State
    }

    /* --------------------> Get Module Position <-------------------- */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(Units.rotationsToRadians(getTurningPositionRot())));
    }

    /* --------------------> Get Module State <-------------------- */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(Units.rotationsToRadians(getTurningPositionRot())));
    }

    /* --------------------> Get Drive Motor Position As Meters <-------------------- */
    public double getDrivePosition(){
        return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.driveMotorRot2Meter;
    }

    /* --------------------> Get Drive Motor Velocity as Meters Per Second <-------------------- */
    public double getDriveVelocity(){
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.driveVelocity2MeterPerSec; 
    }

    /* --------------------> Get Turning Motor Position as Rotations <-------------------- */
    public double getTurningPositionRot(){
        return turnCanCoder.getPosition().getValueAsDouble();
    }

    /* --------------------> Updating Module CANCoder Values to Smart Dashboard <-------------------- */
    @Override
    public void periodic() {
        SmartDashboard.putNumber(moduleName + "CANcoder Position", turnCanCoder.getPosition().getValueAsDouble());
    }

    /* --------------------> Resetting Internal Drive Encoders <-------------------- */
    public void resetEncoders(){
        driveMotor.setPosition(0);
        turnMotor.setPosition(turnCanCoder.getPosition().getValueAsDouble());
    }

    /* --------------------> Stop Swerve Module Motors <-------------------- */
    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /* --------------------> Config Drive Motor <-------------------- */
    private void configDriveMotor(TalonFX driveMotor) {
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.Audio.BeepOnBoot = true;
        driveMotorConfig.Audio.BeepOnConfig = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        driveMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        driveMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
        driveMotorConfig.Feedback.SensorToMechanismRatio = ModuleConstants.driveMotorGearRatio;
        driveMotorConfig.MotorOutput.Inverted = driveMotorDirection;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.Slot0.kP = 0.5;

        TalonFXConfigurator driveMotorConfigurator = driveMotor.getConfigurator();
        driveMotorConfigurator.apply(driveMotorConfig, 0.1);
    }
    
    /* --------------------> Config Turn Motor <-------------------- */
    private void configTurnMotor(TalonFX turnMotor) {
        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
        turnMotorConfig.Audio.BeepOnBoot = true;
        turnMotorConfig.Audio.BeepOnConfig = true;
        turnMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        turnMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        turnMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
        turnMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; // TODO: Maybe fused CANcoder in the future
        turnMotorConfig.Feedback.FeedbackRemoteSensorID = turnCanCoderId;
        turnMotorConfig.Feedback.SensorToMechanismRatio = ModuleConstants.turnMotorGearRatio;
        turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnMotorConfig.Slot0.kP = 0.5;
    
        TalonFXConfigurator turnMotorConfigurator = turnMotor.getConfigurator();
        turnMotorConfigurator.apply(turnMotorConfig, 0.1);
    }

    /* --------------------> Config Turn CANCoder <-------------------- */
    private void configTurnCanCoder(CANcoder turnCanCoder){
        CANcoderConfiguration turnCanCoderConfig = new CANcoderConfiguration();
        turnCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        turnCanCoderConfig.MagnetSensor.MagnetOffset = turnCanCoderOffset;
        turnCanCoderConfig.MagnetSensor.SensorDirection = turnCanCoderDirection;

        CANcoderConfigurator turnCanCoderConfigurator = turnCanCoder.getConfigurator();
        turnCanCoderConfigurator.apply(turnCanCoderConfig, 0.1);
    }
}
