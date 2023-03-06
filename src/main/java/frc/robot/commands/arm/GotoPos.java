package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmPathGenerator;

public class GotoPos extends CommandBase {
    Vector2 target;
    ArmPathGenerator generator;
    ArmPathGenerator.RobotPos pos;
    ArmPathGenerator.RobotOrientation orientation;

    public GotoPos(Vector2 target, ArmPathGenerator.RobotPos pos, ArmPathGenerator.RobotOrientation orientation) {
        this.target = target;
        generator = new ArmPathGenerator();
        this.pos = pos;
        this.orientation = orientation;
    }

    CommandBase generatedPath = null;
    @Override
    public void initialize() {
        generatedPath = ArmPathGenerator.toCommand(new ArmPathGenerator().pathTo(target, pos, orientation));
        CommandScheduler.getInstance().schedule(generatedPath);
    }
    
    @Override
    public void end(boolean interrupted) {
        generatedPath = null;
    }
    
    @Override
    public boolean isFinished() {
        if (generatedPath == null) {
            return true;
        }
        
        return CommandScheduler.getInstance().isScheduled(generatedPath);
    }
}
