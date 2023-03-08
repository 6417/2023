package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmPathGenerator;
import frc.robot.ArmPathGenerator.RobotPos;
import frc.robot.subsystems.Arm;

public class GotoPos extends CommandBase {
    Vector2 target;
    ArmPathGenerator generator;
    ArmPathGenerator.RobotPos pos;
    ArmPathGenerator.RobotOrientation orientation;
    String name;

    public GotoPos(Vector2 target, ArmPathGenerator.RobotPos pos, ArmPathGenerator.RobotOrientation orientation,
            String name) {
        this.target = target;
        generator = new ArmPathGenerator();
        this.pos = pos;
        this.orientation = orientation;
        this.name = name;
    }

    CommandBase generatedPath = null;

    @Override
    public void initialize() {
        if (pos == RobotPos.FIELD) {
            generatedPath = ArmPathGenerator.toCommand(new ArmPathGenerator().pathTo(target,
                    Arm.getInstance().getRobotPos(), Arm.getInstance().getRobotOrientation()));
        } else {
            generatedPath = ArmPathGenerator.toCommand(new ArmPathGenerator().pathTo(target, pos, orientation));
        }
        CommandScheduler.getInstance().schedule(generatedPath);
        System.out.println("[Arm] going to target: " + name + "(" + target + ")");
        Arm.getInstance().setRobotOrientation(orientation);
        Arm.getInstance().setRobotPos(pos);
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
