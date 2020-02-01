/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.HighShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Jevois;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanelMan;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final BringClimberUp m_autoCommand = new BringClimberUp(320);
  public static final LogitechGamingPad gamepad = new LogitechGamingPad(0);
  public static final DriveTrain drive = new DriveTrain();
  public static final Jevois jevois = new Jevois();
  public static final Climber climber = new Climber();
  public static final ControlPanelMan cp_Man = new ControlPanelMan();
  public static final HighShooter highShooter = new HighShooter();
  public final static Intake intake = new Intake();

  public JoystickButton driveA;
  public JoystickButton driveB;
  public JoystickButton driveX;
  public JoystickButton driveY;
  public JoystickButton driveRightBumper;
  public JoystickButton driveLeftBumper;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();

    jevois.initializeSerialPort();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveA = new JoystickButton(gamepad, 1);
    //driveA.whenPressed(new GamepadSlowModeDrive());

    driveB = new JoystickButton(gamepad, 2);
    //driveB.whenPressed();

    driveX = new JoystickButton(gamepad, 3);
    driveX.whenPressed(new DriveDistance(0));

    driveY = new JoystickButton(gamepad, 4);
    //driveY.whenPressed();
    
    driveRightBumper = new JoystickButton(gamepad, 6);
    //driveRightBumper.whenPressed(new GoToGoalColor(DriverStation.getInstance().getGameSpecificMessage().substring(0, 1)));
    driveRightBumper.whenPressed(new Shoot());

    driveLeftBumper = new JoystickButton(gamepad, 5);
    driveLeftBumper.whenPressed(new IntakeBalls());
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
