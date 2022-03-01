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

public class RotationControl extends CommandBase {
  /**
   * Creates a new RotationControl.
   */
  int counter;
  int colorCounter;
  Timer timer = new Timer();
  public RotationControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.cp_Man);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // RobotContainer.cp_Man.run(.35);
    counter = 0;
    colorCounter = 0;
    timer.reset();
    timer.start();
  }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
   
  //   if (RobotContainer.cp_Man.getColor().equals("R")) {
  //     counter += 1;
  //       if (counter == 3) {
  //         counter += 1;
  //         colorCounter += 1;
  //         System.out.println(timer.get());
  //         timer.reset();
  //       }
  //   }
  //   else counter = 0; 
    
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.cp_Man.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (colorCounter > 5);
  }
}
