package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.fridowpi.utils.Vector2;
import frc.robot.ArmPosJoystick;
import frc.robot.ArmPathGenerator.RobotOrientation;
import frc.robot.ArmPathGenerator.RobotPos;
import frc.robot.ArmPosJoystick.Ids;
import frc.robot.commands.arm.GotoPos;
import frc.robot.subsystems.Arm;

public class ArmControllCommands {
    private static class GotoPosAutonomous extends GotoPos {

        public GotoPosAutonomous(Vector2 target, RobotPos pos, RobotOrientation orientation, String name) {
            super(target, pos, orientation, name);
            System.out.println("[GotoPosAutonomous::new]");
        }

        @Override
        public void initialize() {
            super.initialize();
            System.out.println("[GotoPosAutonomous::initialize]");
        }

        @Override
        public boolean isFinished() {
            boolean superFinished = super.isFinished();

            System.out.println("[GotoPosAutonomous::isFinished] generatedPath: " + super.generatedPath
                    + ", super isFinished: " + superFinished + ", this isFinished: "
                    + (Arm.getInstance().getPos().minus(target).magnitude() < 3.0));
            System.out.println("[GotoPosAutonomous::isFinished] current pos: " + Arm.getInstance().getPos()
                    + ", target: " + super.target + ", error: " + Arm.getInstance().getPos().minus(target).magnitude());
            if (super.generatedPath == null) {
                return true;
            }

            return Arm.getInstance().getPos().minus(target).magnitude() < 0.03 || !Arm.getInstance().isZeroed();
        }

    }

    public static interface CommandFactory {
        CommandBase construct();
    }

    private static final ArmPosJoystick.Ids homeParams = Ids.HOME;
    // ----------------------------------------------------------------------------------------------------------------
    private static final ArmPosJoystick.Ids gridForwardTopParams = Ids.GRID_FORWARD_TOP;
    private static final ArmPosJoystick.Ids gridForwardMiddleParams = Ids.GRID_FORWARD_MIDDLE;
    private static final ArmPosJoystick.Ids gridForwardBottomParams = Ids.GRID_FORWARD_BOTTOM;

    private static final ArmPosJoystick.Ids gridReverseMiddleParams = Ids.GRID_REVERSE_MIDDLE;
    private static final ArmPosJoystick.Ids gridReverseBottomParams = Ids.GRID_FORWARD_BOTTOM;
    // ----------------------------------------------------------------------------------------------------------------
    private static final ArmPosJoystick.Ids loadingZoneForwardConeParams = Ids.LOADING_ZONE_FORWARD_CONE;
    private static final ArmPosJoystick.Ids loadingZoneForwardCubeParams = Ids.LOADING_ZONE_FORWARD_CUBE;

    private static final ArmPosJoystick.Ids loadingZoneReverseConeParams = Ids.LOADING_ZONE_REVERSE_CONE;
    private static final ArmPosJoystick.Ids loadingZoneReverseCubeParams = Ids.LOADING_ZONE_REVERSE_CUBE;
    // ----------------------------------------------------------------------------------------------------------------
    private static final ArmPosJoystick.Ids pickUpForwardParams = Ids.PICKUP_FORWARD;
    private static final ArmPosJoystick.Ids pickUpReverseParams = Ids.PICKUP_REVERSE;

    // ################################################################################################################

    public static CommandFactory home = () -> new GotoPosAutonomous(homeParams.target, homeParams.pos,
            homeParams.orientation,
            homeParams.toString());

    public static CommandFactory gridForwardTop = () -> new GotoPosAutonomous(gridForwardTopParams.target,
            gridForwardTopParams.pos, gridForwardTopParams.orientation, gridForwardTopParams.toString());
    public static CommandFactory gridForwardMiddle = () -> new GotoPosAutonomous(gridForwardMiddleParams.target,
            gridForwardMiddleParams.pos, gridForwardMiddleParams.orientation, gridForwardMiddleParams.toString());
    public static CommandFactory gridForwardBottom = () -> new GotoPosAutonomous(gridForwardBottomParams.target,
            gridForwardBottomParams.pos, gridForwardBottomParams.orientation, gridForwardBottomParams.toString());

    public static CommandFactory gridReverseMiddle = () -> new GotoPosAutonomous(gridReverseMiddleParams.target,
            gridReverseMiddleParams.pos, gridReverseMiddleParams.orientation, gridReverseMiddleParams.toString());
    public static CommandFactory gridReverseBottom = () -> new GotoPosAutonomous(gridReverseBottomParams.target,
            gridReverseBottomParams.pos, gridReverseBottomParams.orientation, gridReverseBottomParams.toString());

    public static CommandFactory loadingZoneForwardCone = () -> new GotoPosAutonomous(
            loadingZoneForwardConeParams.target,
            loadingZoneForwardConeParams.pos, loadingZoneForwardConeParams.orientation,
            loadingZoneForwardConeParams.toString());
    public static CommandFactory loadingZoneForwardCube = () -> new GotoPosAutonomous(
            loadingZoneForwardCubeParams.target,
            loadingZoneForwardCubeParams.pos, loadingZoneForwardCubeParams.orientation,
            loadingZoneForwardCubeParams.toString());

    public static CommandFactory loadingZoneReverseCone = () -> new GotoPosAutonomous(
            loadingZoneReverseConeParams.target,
            loadingZoneReverseConeParams.pos, loadingZoneReverseConeParams.orientation,
            loadingZoneReverseConeParams.toString());
    public static CommandFactory loadingZoneReverseCube = () -> new GotoPosAutonomous(
            loadingZoneReverseCubeParams.target,
            loadingZoneReverseCubeParams.pos, loadingZoneReverseCubeParams.orientation,
            loadingZoneReverseCubeParams.toString());

    public static CommandFactory pickUpForward = () -> new GotoPosAutonomous(pickUpForwardParams.target,
            pickUpForwardParams.pos,
            pickUpForwardParams.orientation, pickUpForwardParams.toString());
    public static CommandFactory pickUpReverse = () -> new GotoPosAutonomous(pickUpReverseParams.target,
            pickUpReverseParams.pos,
            pickUpReverseParams.orientation, pickUpReverseParams.toString());
}
