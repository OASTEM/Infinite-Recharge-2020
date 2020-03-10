/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RampRateChange extends CommandBase {
  /**
   * Creates a new RampRateChange.
   */
  boolean down;
  double rate;
  public RampRateChange() {
    addRequirements(RobotContainer.drive);
    down = true;
    rate = Constants.dOpenLoop_Ramp;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (rate > 0.2) {
      down = true;
    }
    else down = false;

    if (down) {
      rate -= .1;
    }
    else rate += .1;
    
    RobotContainer.drive.setOpenRampRate(rate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
