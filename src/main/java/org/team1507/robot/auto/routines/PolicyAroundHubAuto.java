//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS


package org.team1507.robot.auto.routines;

import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.opmode.Autonomous;

import org.team1507.lib.core.framework.AutoOpMode;
import org.team1507.robot.auto.AutoSequence;
import org.team1507.robot.auto.Driver;
import org.team1507.robot.auto.nodes.Node;
import org.team1507.robot.auto.nodes.Nodes;

// ─────────────────────────────────────────────────────────────────────────────
// PolicyAroundHubAuto  (1507Labs exp/policy-driver demo)
//
// A lap of the hub from the right start: behind the hub, over to the left
// start, then back to the right scoring spot facing the hub. The first two
// legs cut across the hub's corners, so the policy has to steer around them.
// A classic version would need extra nodes at each corner.
// ─────────────────────────────────────────────────────────────────────────────
@Autonomous(name = "Policy - Around the Hub")
public final class PolicyAroundHubAuto extends AutoOpMode {

    /** Behind the hub, on the neutral-zone side. */
    private static final Pose2d BEHIND_HUB = Node.at(6.0, 4.035, 90.0);

    @Override
    protected Command build() {
        return new AutoSequence(Driver.POLICY)
            .resetPose(Nodes.Robot.Start.RIGHT)
            .checkpoint(BEHIND_HUB)
            .checkpoint(Nodes.Robot.Start.LEFT)
            .endpoint(Nodes.Robot.Score.RIGHT).facing(Nodes.FieldElements.Hub.CENTER)
            .build();
    }
}
