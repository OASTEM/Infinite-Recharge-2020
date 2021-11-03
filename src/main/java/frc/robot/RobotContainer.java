/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.Auto.Left;
import frc.robot.commands.Auto.Middle;
import frc.robot.commands.Auto.Right;
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
  public SendableChooser<String> chooser;
  private final Left l_autoCommand = new Left();
  private final Middle m_autoCommand = new Middle();
  private final Right r_autoCommand = new Right();
  private final String straightAuto = "StraightAuto";
  private final String leftAuto = "LeftAuto";
  private final String middleAuto = "MiddleAuto";
  private final String rightAuto = "RightAuto";

  public static final LogitechGamingPad drivePad = new LogitechGamingPad(0);
  public static final LogitechGamingPad opPad = new LogitechGamingPad(1);
  public static final DriveTrain drive = new DriveTrain();
  public static final Jevois jevois = new Jevois();
  public static final Climber climber = new Climber();
  public static final ControlPanelMan cp_Man = new ControlPanelMan();
  public static final HighShooter highShooter = new HighShooter();
  public static final LowDumper lowDumper = new LowDumper();
  public static final NavX navX = new NavX();
  public static final Camera axisCam = new Camera();

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

  public Joystick attacStick = new Joystick(2);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();

    jevois.initializeSerialPort();

    drive.setDefaultCommand(new GamepadDrive());
    climber.setDefaultCommand(new GamepadClimb());

    chooser = new SendableChooser<String>();
    chooser.setDefaultOption("Straight Auto", straightAuto);
    chooser.addOption("Left Auto", leftAuto);
    chooser.addOption("Middle Auto", middleAuto);
    chooser.addOption("Right Auto", rightAuto);

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveA = new JoystickButton(drivePad, 1);
    driveA.whileHeld(new ShootHigh());
    //driveA.whenPressed(new drivePadSlowModeDrive());
    //driveA.whenPressed(new RampRateChange());

    driveB = new JoystickButton(drivePad, 2);
    //driveB.whenPressed(new DriveDistanceCurve(90, 10));
//19 inches per rot

    driveX = new JoystickButton(drivePad, 3);
    driveX.whileHeld(new OuttakeBalls(0.3, false));

    driveY = new JoystickButton(drivePad, 4);
    //driveY.whenPressed(new ChangeDriveMode());
    driveY.whileHeld(new IntakeBalls());
    
    driveRightBumper = new JoystickButton(drivePad, 6);
    driveRightBumper.whenPressed(new GamepadDrive());

    driveLeftBumper = new JoystickButton(drivePad, 5);
    driveLeftBumper.whenPressed(new GamepadSlowModeDrive()) ;

    driveStart = new JoystickButton(drivePad, 8);
    driveStart.whenHeld(new SelfTestCommand());
  
    opA = new JoystickButton(opPad, 1);
    opA.whenPressed(new AdjustClimber());

    opB = new JoystickButton(opPad, 2);
    //opB.whenPressed(command);

    opX = new JoystickButton(opPad, 3);
    opX.whenHeld(new BringClimberDown());

    opY = new JoystickButton(opPad, 4);
    opY.whenHeld(new BringClimberUp());

    opLeftBumper = new JoystickButton(opPad, 5);
    opLeftBumper.whenPressed(new RotationControl());

    opRightBumper = new JoystickButton(opPad, 6);
    //opRightBumper.whenPressed(new GoToGoalColor(DriverStation.getInstance().getGameSpecificMessage()));
    opRightBumper.whenPressed(new GoToGoalColorGroup(DriverStation.getInstance().getGameSpecificMessage()));
  
    JoystickButton trigger = new JoystickButton(attacStick, 1);
    trigger.whileHeld(new GroundIntake());

    JoystickButton left = new JoystickButton(attacStick, 4);
    left.whileHeld(new IntakeBalls());

    JoystickButton right = new JoystickButton(attacStick, 5);
    right.whileHeld(new OuttakeBalls(0.3, false));
  }





  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    if(chooser.getSelected().equals(leftAuto)) {
      return l_autoCommand;
    }
    else if(chooser.getSelected().equals(middleAuto)) {
      return m_autoCommand;
    }
    else if(chooser.getSelected().equals(rightAuto)) {
      return r_autoCommand;
    }
    else {
      return new DriveDistance(95, "B", 0);
      //return m_autoCommand;
    }
  }
}
