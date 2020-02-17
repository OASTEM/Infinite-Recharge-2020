/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
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
  private final BringClimberUp m_autoCommand = new BringClimberUp();
  public static final LogitechGamingPad drivePad = new LogitechGamingPad(0);
  public static final LogitechGamingPad opPad = new LogitechGamingPad(1);
  public static final DriveTrain drive = new DriveTrain();
  //public static final DriveTrainV2 drive = new DriveTrainV2();
  public static final Jevois jevois = new Jevois();
  public static final Climber climber = new Climber();
  public static final ControlPanelMan cp_Man = new ControlPanelMan();
  public static final HighShooter highShooter = new HighShooter();
  public static final Intake intake = new Intake();
  public static final NavX navX = new NavX();

  public JoystickButton driveA;
  public JoystickButton driveB;
  public JoystickButton driveX;
  public JoystickButton driveY;
  public JoystickButton driveStart;
  public JoystickButton driveRightBumper;
  public JoystickButton driveLeftBumper;
  
  public JoystickButton opA;
  public JoystickButton opB;
  public JoystickButton opX;
  public JoystickButton opY;
  public JoystickButton opStart;
  public JoystickButton opLeftBumper;
  public JoystickButton opRightBumper;

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
    driveA = new JoystickButton(drivePad, 1);
    //driveA.whenPressed(new drivePadSlowModeDrive());

    driveB = new JoystickButton(drivePad, 2);
    driveB.whenPressed(new BringClimberUp());

    driveX = new JoystickButton(drivePad, 3);
    driveX.whenPressed(new DriveDistance(50));

    driveY = new JoystickButton(drivePad, 4);
    //driveY.whenPressed();
    
    driveRightBumper = new JoystickButton(drivePad, 6);
    driveRightBumper.whenPressed(new GoToGoalColor("R"));
    //driveRightBumper.whenPressed(new GoToGoalColor(DriverStation.getInstance().getGameSpecificMessage().substring(0, 1)));
    //driveRightBumper.whenPressed(new Shoot());

    driveLeftBumper = new JoystickButton(drivePad, 5);
    driveLeftBumper.whenPressed(new IntakeBalls());

    driveStart = new JoystickButton(drivePad, 8);
    driveStart.whenPressed(new SelfTestCommand());
  
    opA = new JoystickButton(opPad, 1);
    //opA.whenPressed(new GoToGoalColor("R"));

    opB = new JoystickButton(opPad, 2);
    //opB.whenPressed(command);

    opX = new JoystickButton(opPad, 3);
    //opX.whenPressed(command);

    opY = new JoystickButton(opPad, 4);
    //opY.whenPressed(command);

    opLeftBumper = new JoystickButton(opPad, 5);
    //opLeftBumper.whenPressed(command);

    opRightBumper = new JoystickButton(opPad, 6);
    //opRightBumper.whenPressed(command);
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
