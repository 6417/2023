package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmPathGenerator;
import frc.robot.ArmPathGenerator.RobotPos;
import frc.robot.subsystems.Arm;

public class GotoPos extends CommandBase {
    protected final Vector2 target;
    protected final ArmPathGenerator generator;
    protected final ArmPathGenerator.RobotPos pos;
    protected final ArmPathGenerator.RobotOrientation orientation;
    protected final String name;
    protected final boolean invertXDir;

    public GotoPos(Vector2 target, ArmPathGenerator.RobotPos pos, ArmPathGenerator.RobotOrientation orientation,
            String name, boolean invertXDir) {
        this.target = target;
        generator = new ArmPathGenerator();
        this.pos = pos;
        this.orientation = orientation;
        this.name = name;
        this.invertXDir = invertXDir;
    }

    protected CommandBase generatedPath = null;
    Timer debug = new Timer();

    @Override
    public void initialize() {
        Arm.getInstance().xDirInverted = invertXDir;
        debug.start();
        if (pos == RobotPos.FIELD) {
            Vector2[] generatedTargets = new ArmPathGenerator().pathTo(target,
                    Arm.getInstance().getRobotPos(), Arm.getInstance().getRobotOrientation());

            for (var p : generatedTargets) {
                System.out.println(p);
            }

            generatedPath = ArmPathGenerator.toCommand(generatedTargets);
        } else {
            Vector2[] generatedTargets = new ArmPathGenerator().pathTo(target, pos, orientation);
            if (generatedTargets.length == 0) {
                return;
            }
            for (var p : generatedTargets) {
                System.out.println(p);
            }
            generatedPath = ArmPathGenerator.toCommand(generatedTargets);

        }
        CommandScheduler.getInstance().schedule(generatedPath);
        System.out.println("(Time: " + debug.get() + ") [Arm] going to target: " + name + "(" + target + ")");

        Arm.getInstance().setRobotOrientation(orientation);
        Arm.getInstance().setRobotPos(pos);
    }

    @Override
    public void end(boolean interrupted) {
        generatedPath = null;
        System.out.println("(Time: " + debug.get() + ") [Arm] ended, (interrupted: " + interrupted + ") target: " + name
                + "(" + target + ")");
    }

    @Override
    public boolean isFinished() {
        if (generatedPath == null) {
            return true;
        }

        return CommandScheduler.getInstance().isScheduled(generatedPath) || !Arm.getInstance().isZeroed();
    }
}
