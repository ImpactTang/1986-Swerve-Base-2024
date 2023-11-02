package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.swerve.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  /* --------------------> Subsystems <-------------------- */
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  /* --------------------> Controllers <-------------------- */
  private final CommandXboxController cmdDriveController = new CommandXboxController(IOConstants.kDriveControllerPort);
  private final CommandJoystick buttonBox = new CommandJoystick(IOConstants.kButtonBoxPort);

  public RobotContainer() {

    /* --------------------> Swerve Controller Drive <-------------------- */
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
    () -> cmdDriveController.getLeftX(), 
    () -> cmdDriveController.getLeftY(), 
    () -> cmdDriveController.getRightX(), 
    () -> cmdDriveController.x().getAsBoolean()));

    // Configure Button Bindings
    configureBindings();
  }

  private void configureBindings() {

    /* --------------------> Swerve Buttons <-------------------- */
    cmdDriveController.y().onTrue(new InstantCommand(() -> swerveSubsystem.setHeading(0)));
    cmdDriveController.a().onTrue(new InstantCommand(() -> swerveSubsystem.setHeading(180)));

  }

  /* --------------------> Get Autonomous Command <-------------------- */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
