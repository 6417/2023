package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class BaseGotoPositionShuffleBoard extends CommandBase {
    private static BaseGotoPositionShuffleBoard instance = null;

    public double target = Math.PI / 2.0;

    private BaseGotoPositionShuffleBoard() {
        addRequirements(Arm.getInstance());
    }
    
    public static BaseGotoPositionShuffleBoard getInstance() {
          if (instance == null) {
            instance = new BaseGotoPositionShuffleBoard();
          }
          return instance;
    }
    
    
    @Override
    public void initialize() {
        Arm.getInstance().baseGotoAngle(target);
    }
    
    @Override
    public void end(boolean interrupted) {
        Arm.getInstance().stop(); 
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
