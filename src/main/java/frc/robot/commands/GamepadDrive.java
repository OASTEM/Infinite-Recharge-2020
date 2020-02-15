/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogitechGamingPad;
import frc.robot.RobotContainer;

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

    time = timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive.drivePercentOutput(RobotContainer.drivePad.getLeftAnalogY(), RobotContainer.drivePad.getRightAnalogY());
    /*if(RobotContainer.drive.getLeftMotorOutput() + RobotContainer.drive.getRightMotorOutput() <= 0.1) {
      timer.reset();
    }
    else {
      if(time == .5){
        RobotContainer.drive.stop();
        Timer.delay(0.1);
        RobotContainer.drive.drivePercentOutput(RobotContainer.drivePad.getLeftAnalogY(), RobotContainer.drivePad.getRightAnalogY());
      }
      else if(time == 1) {
        RobotContainer.drive.stop();
        Timer.delay(0.1);
        RobotContainer.drive.drivePercentOutput(RobotContainer.drivePad.getLeftAnalogY(), RobotContainer.drivePad.getRightAnalogY());
      }
      else if(time == 2.5) {
        RobotContainer.drive.stop();
        Timer.delay(0.1);
        RobotContainer.drive.drivePercentOutput(RobotContainer.drivePad.getLeftAnalogY(), RobotContainer.drivePad.getRightAnalogY());
      }
      else{
        double multi = 1 - (RobotContainer.drivePad.getLeftTriggerValue() * .5);
        RobotContainer.drive.drivePercentOutput(RobotContainer.drivePad.getLeftAnalogY() * multi, RobotContainer.drivePad.getRightAnalogY() * multi);
      }
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
