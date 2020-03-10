/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class OuttakeBalls extends CommandBase {
  /**
   * Creates a new OuttakeBalls.
   */
  double timeout;
  Timer timer;
  boolean isAuto;
  
  public OuttakeBalls(double timeout, boolean isAuto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.timeout = timeout;
    this.isAuto = isAuto;
    addRequirements(RobotContainer.lowDumper);
    addRequirements(RobotContainer.drive);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.lowDumper.outtake();
    if(isAuto) {
      RobotContainer.drive.drivePercent(-.15, -.15);
    }
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.lowDumper.stop();
    RobotContainer.drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > timeout;
  }
}
