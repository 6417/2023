package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmCalculator;
import frc.robot.Constants;
import frc.robot.MotionMagic;
import frc.robot.subsystems.Arm;

public class HoldAtCurrentPosition extends CommandBase {
    private MotionMagic mmController;

    public HoldAtCurrentPosition() {
        addRequirements(Arm.getInstance());
        mmController = new MotionMagic(1.0, ArmCalculator.ampsToTorque(10) / Constants.Arm.baseGearRatio,
                ArmCalculator.ampsToTorque(4) / Constants.Arm.baseGearRatio,
                () -> Arm.getInstance().baseAngle() * Constants.Arm.baseArm.centerOfMass);
    }

    private double currentPos;

    @Override
    public void initialize() {
        currentPos = Arm.getInstance().baseAngle();
        mmController.goTo(Math.PI / 4 * Constants.Arm.baseArm.centerOfMass);

    }

    boolean reached = false;

    @Override
    public void execute() {
        var torques = Arm.getInstance().getCalculator().calculateTorquesForStall();

        double mmOut = mmController.getAccel() * Constants.Arm.baseGearRatio;

        if (!reached) {
            Arm.getInstance().setBaseAmpsLimit(ArmCalculator.torqueToAmps(torques.base + mmOut));
            Arm.getInstance().setPercentBase(Math.signum(torques.base + mmOut));
        } else {

            Arm.getInstance().setBaseAmpsLimit(ArmCalculator.torqueToAmps(torques.base));
            Arm.getInstance().setPercentBase(Math.signum(torques.base));
        }

        if (Math.abs(Arm.getInstance().baseAngle() - Math.PI / 4) < 0.01) {
            reached = true;
        }
        
        // System.out.print("error: ");
        // System.out.println(Math.abs(Arm.getInstance().baseAngle() - Math.PI / 4));
    }
}
