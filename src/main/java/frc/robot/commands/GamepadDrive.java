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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GamepadDrive extends CommandBase {
  /**
   * Creates a new GamepadDrive.
   */

  Timer timer;
  double time;
  LogitechGamingPad drivePad;

  public GamepadDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
    drivePad = RobotContainer.drivePad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();

    //System.out.println(Robot.driveConstant);

    /*if(Robot.driveConstant == Constants.regMode) {
      Robot.driveConstant = Constants.slowMode;
      System.out.println("changed to slow");
    }
    else {
      Robot.driveConstant = Constants.regMode;
      System.out.println("changed to reg");
    }*/

  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("DriveTrain Left Pos", RobotContainer.drive.getLeftPosition());
    SmartDashboard.putNumber("DriveTrain Right Pos", RobotContainer.drive.getRightPosition());
    double leftJoystick = RobotContainer.drivePad.getLeftAnalogY();
    double rightJoystick = RobotContainer.drivePad.getRightAnalogY();
    RobotContainer.drive.drivePercent(Robot.driveConstant * leftJoystick, Robot.driveConstant * rightJoystick);
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
