package org.firstinspires.ftc.teamcode.util.auto_util;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.StateMachine;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

public final class AutoRoutines {
    private static final PoseFactory poseFactory = PoseFactory.degrees();

    public static class Test1 {

        public static final Pose test1start = poseFactory.of(83, 134, 90);
        private static final Pose t1P1 = poseFactory.of(82.7101, 110.4067, -90);

        public static Path t1P1() {
            return line(test1start, t1P1).constant(t1P1);
        }

        public static Command run(Robot robot) {
            return sequential(
                    robot.drivetrain.startPathCmd(t1P1()),
                    waitUntil(() -> robot.drivetrain.getState() != Drivetrain.DrivetrainState.FOLLOWING),
                    robot.fsm.requestStateCmd(StateMachine.State.SHOOT_POLLEN),
                    waitUntil(() -> robot.shooter.isReady()
                            && robot.transfer.transferState == Transfer.TransferState.RUNNING),
                    waitMs(300),
                    robot.fsm.requestStateCmd(StateMachine.State.IDLE)
            );
        }
    }
}
