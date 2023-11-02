package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    /* --------------------> Swerve Drive Constants <-------------------- */
    public static final class DriveConstants{

        /* --------------------> Robot Dimensions <-------------------- */
        public static final double kTrackWidth = 16; // Distance between centers of right and left wheels on robot (Width, X)
        public static final double kWheelBase = 23.5; // Distance between centers of front and back wheels on robot (Length, Y)

        /* --------------------> Robot Max Speeds <-------------------- */
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0; // Physical Max Robot Speed for Swerve Drive Kinematics
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI; // Physical Max Robot Turning Speed
        public static final double kMaxAccelerationRateUnitsPerSecond = 5.0; // Max Acceleration Rate of Robot (Used in SlewRateLimiter)
        public static final double kMaxTurningRateUnitsPerSecond = 5.0; // Max Turning Acceleration Rate of Robot (Used in SlewRateLimiter)

        public static final double kDriveMaxSpeedMetersPerSecond = 5.0; // Max Speed of Robot
        public static final double kDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 8; // Max Turning Speed of Robot
    }

    /* --------------------> Swerve Module Constants <-------------------- */
    public static final class ModuleConstants {
        public static final double wheelDiameterMeters = Units.inchesToMeters(4);
        public static final double driveMotorGearRatio = 6.86;
        public static final double turnMotorGearRatio = 12.8;
        public static final double driveMotorRot2Meter = (Math.PI * wheelDiameterMeters) / driveMotorGearRatio;
        public static final double driveVelocity2MeterPerSec = driveMotorRot2Meter / 60;

        /* --------------------> Front Left Module <-------------------- */
        public static final class FrontLeftModule {
            public static final int driveMotorId = 4;
            public static final int turnMotorId = 5;
            public static final InvertedValue driveMotorDirection = InvertedValue.Clockwise_Positive;
            public static final int turnCanCoderId = 12;
            public static final SensorDirectionValue turnCanCoderDirection = SensorDirectionValue.Clockwise_Positive;
            public static final double turnCanCoderOffset = 0.0 / 360.0;
            public static final String moduleName = "Front Left";
        }

        /* --------------------> Front Right Module <-------------------- */
        public static final class FrontRightModule {
            public static final int driveMotorId = 0;
            public static final int turnMotorId = 1;
            public static final InvertedValue driveMotorDirection = InvertedValue.CounterClockwise_Positive;
            public static final int turnCanCoderId = 10;
            public static final SensorDirectionValue turnCanCoderDirection = SensorDirectionValue.Clockwise_Positive;
            public static final double turnCanCoderOffset = 0.0 / 360.0;
            public static final String moduleName = "Front Right";
        }

        /* --------------------> Back Left Module <-------------------- */
        public static final class BackLeftModule {
            public static final int driveMotorId = 6;
            public static final int turnMotorId = 7;
            public static final InvertedValue driveMotorDirection = InvertedValue.Clockwise_Positive;
            public static final int turnCanCoderId = 13;
            public static final SensorDirectionValue turnCanCoderDirection = SensorDirectionValue.Clockwise_Positive;
            public static final double turnCanCoderOffset = 0.0 / 360.0;
            public static final String moduleName = "Back Left";
        }

        /* --------------------> Back Right Module <-------------------- */
        public static final class BackRightModule {
            public static final int driveMotorId = 2;
            public static final int turnMotorId = 3;
            public static final InvertedValue driveMotorDirection = InvertedValue.CounterClockwise_Positive;
            public static final int turnCanCoderId = 11;
            public static final SensorDirectionValue turnCanCoderDirection = SensorDirectionValue.Clockwise_Positive;
            public static final double turnCanCoderOffset = 0.0 / 360.0;
            public static final String moduleName = "Back Right";
        }
    }

    /* --------------------> IO Constants <-------------------- */
    public static final class IOConstants {

        /* --------------------> Controller Ports <-------------------- */
        public static final int kDriveControllerPort = 0;
        public static final int kButtonBoxPort = 1;

        /* --------------------> Joystick Deadband <-------------------- */
        public static final double kJoystickDeadband = 0.05;

        /* --------------------> Button Box Buttons <-------------------- */
        public static final class ButtonBoxButtons {
            public static final int straightUpButton = 1;
            public static final int armRotateButton = 2;
            public static final int frontSwitch = 3;
            public static final int floorSwitch = 4;
            public static final int highSwitch = 5;
            public static final int tippedSwitch = 6;
            public static final int cubeSwitch = 7;
            public static final int singleSubstation = 8;
            public static final int parallelButton = 9;
            public static final int doubleSubstation = 10;
            public static final int jogDownSwitch = 11;
            public static final int jogUpSwitch = 12;
            public static final int knobAxis = 0;
        }
    }
}
