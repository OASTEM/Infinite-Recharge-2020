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

public class BringClimberUp extends CommandBase {
  /**
   * Creates a new RunFrontLeg.
   */
  public BringClimberUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.reset();
    Timer.delay(0.1);
    RobotContainer.climber.set(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Bring Climber up - execute()");
    SmartDashboard.putNumber("Position", RobotContainer.climber.getFrontPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.climber.getFrontPosition() >= 320) {
      return true;
    }
    return false;
  }
}
