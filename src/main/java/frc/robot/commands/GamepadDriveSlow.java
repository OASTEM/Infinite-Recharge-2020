/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LogitechGamingPad;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class GamepadDriveSlow extends CommandBase {
  /**
   * Creates a new GamepadDriveSlow.
   */

  Timer timer;
  double time;
  LogitechGamingPad drivePad;

  public GamepadDriveSlow() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    addRequirements(RobotContainer.navX);
    drivePad = RobotContainer.drivePad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();

    time = timer.get();

    Robot.driveConstant = Constants.slowMode;
    RobotContainer.navX.reset();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    RobotContainer.drive.drivePercentOutput(Robot.driveConstant * RobotContainer.drivePad.getLeftAnalogY(), Robot.driveConstant * RobotContainer.drivePad.getRightAnalogY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
