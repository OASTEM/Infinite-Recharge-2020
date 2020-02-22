/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveDistance extends CommandBase {
  /**
   * Creates a new DriveDistance.
   */

  private double goal;
  private Timer timer = new Timer();

  public DriveDistance(double goal) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goal = goal;
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    RobotContainer.drive.reset();
    Timer.delay(0.1);
    RobotContainer.drive.drivePosition(goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("DriveTrain Left Pos", RobotContainer.drive.getLeftPosition());
    SmartDashboard.putNumber("DriveTrain Right Pos", RobotContainer.drive.getRightPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      RobotContainer.drive.stop();
    }
    else {
      RobotContainer.drive.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return timer.get() >= 0.5 && (Math.abs(RobotContainer.drive.backLeft.get()) < 0.08 && Math.abs(RobotContainer.drive.backRight.get()) < 0.08);
  }
}
