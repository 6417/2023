package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmPosJoystick;
import frc.robot.ArmPosJoystick.Ids;
import frc.robot.commands.arm.GotoPos;

public class ArmControllCommands {
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

    public static CommandFactory home = () -> new GotoPos(homeParams.target, homeParams.pos, homeParams.orientation,
            homeParams.toString());

    public static CommandFactory gridForwardTop = () -> new GotoPos(gridForwardTopParams.target,
            gridForwardTopParams.pos, gridForwardTopParams.orientation, gridForwardTopParams.toString());
    public static CommandFactory gridForwardMiddle = () -> new GotoPos(gridForwardMiddleParams.target,
            gridForwardMiddleParams.pos, gridForwardMiddleParams.orientation, gridForwardMiddleParams.toString());
    public static CommandFactory gridForwardBottom = () -> new GotoPos(gridForwardBottomParams.target,
            gridForwardBottomParams.pos, gridForwardBottomParams.orientation, gridForwardBottomParams.toString());

    public static CommandFactory gridReverseMiddle = () -> new GotoPos(gridReverseMiddleParams.target,
            gridReverseMiddleParams.pos, gridReverseMiddleParams.orientation, gridReverseMiddleParams.toString());
    public static CommandFactory gridReverseBottom = () -> new GotoPos(gridReverseBottomParams.target,
            gridReverseBottomParams.pos, gridReverseBottomParams.orientation, gridReverseBottomParams.toString());

    public static CommandFactory loadingZoneForwardCone = () -> new GotoPos(loadingZoneForwardConeParams.target,
            loadingZoneForwardConeParams.pos, loadingZoneForwardConeParams.orientation,
            loadingZoneForwardConeParams.toString());
    public static CommandFactory loadingZoneForwardCube = () -> new GotoPos(loadingZoneForwardCubeParams.target,
            loadingZoneForwardCubeParams.pos, loadingZoneForwardCubeParams.orientation,
            loadingZoneForwardCubeParams.toString());

    public static CommandFactory loadingZoneReverseCone = () -> new GotoPos(loadingZoneReverseConeParams.target,
            loadingZoneReverseConeParams.pos, loadingZoneReverseConeParams.orientation,
            loadingZoneReverseConeParams.toString());
    public static CommandFactory loadingZoneReverseCube = () -> new GotoPos(loadingZoneReverseCubeParams.target,
            loadingZoneReverseCubeParams.pos, loadingZoneReverseCubeParams.orientation,
            loadingZoneReverseCubeParams.toString());

    public static CommandFactory pickUpForward = () -> new GotoPos(pickUpForwardParams.target, pickUpForwardParams.pos,
            pickUpForwardParams.orientation, pickUpForwardParams.toString());
    public static CommandFactory pickUpReverse = () -> new GotoPos(pickUpReverseParams.target, pickUpReverseParams.pos,
            pickUpReverseParams.orientation, pickUpReverseParams.toString());
}
