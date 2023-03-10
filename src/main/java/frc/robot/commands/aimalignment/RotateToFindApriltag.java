package frc.robot.commands.aimalignment;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateToFindApriltag extends CommandBase{
    @Override
    public void initialize(){
        System.out.println("finde apriltag");
    }

    @Override 
    public void execute(){
        System.out.println("exec find apriltag");
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
    @Override
    public void end(boolean interrupted){
        System.out.println("end");
    }
    
}
