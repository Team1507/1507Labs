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
// PolicyThroughHubAuto  (1507Labs exp/policy-driver demo)
//
// Both legs go straight THROUGH the hub: start in front of it, drive to the
// spot directly behind it, and come back. The classic driver would hit the hub
// (its route check refuses this auto); the policy steers around it on its own.
// ─────────────────────────────────────────────────────────────────────────────
@Autonomous(name = "Policy - Through the Hub")
public final class PolicyThroughHubAuto extends AutoOpMode {

    /** Directly behind the hub, on the neutral-zone side. */
    private static final Pose2d BEHIND_HUB = Node.at(6.3, 4.1, 180.0);

    @Override
    protected Command build() {
        return new AutoSequence(Driver.POLICY)
            .resetPose(Nodes.Robot.Start.CENTER)
            .checkpoint(BEHIND_HUB)                          // keeps moving: goes around, then back
            .endpoint(Nodes.Robot.Start.CENTER)
            .build();
    }
}
