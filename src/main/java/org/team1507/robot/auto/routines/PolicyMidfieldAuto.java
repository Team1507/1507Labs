//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS


package org.team1507.robot.auto.routines;

import org.wpilib.command3.Command;
import org.wpilib.opmode.Autonomous;

import org.team1507.lib.core.framework.AutoOpMode;
import org.team1507.robot.auto.AutoSequence;
import org.team1507.robot.auto.Driver;
import org.team1507.robot.auto.nodes.Nodes;

// ─────────────────────────────────────────────────────────────────────────────
// PolicyMidfieldAuto  (1507Labs exp/policy-driver demo)
//
// "Example Route" driven by the policy: the same nodes and steps, only
// new AutoSequence(Driver.POLICY) instead of new AutoSequence(). Run the two
// back to back to compare the drivers on the same route.
// ─────────────────────────────────────────────────────────────────────────────
@Autonomous(name = "Policy - Midfield and Back")
public final class PolicyMidfieldAuto extends AutoOpMode {

    @Override
    protected Command build() {
        return new AutoSequence(Driver.POLICY)
            .maxSpeed(0.8)
            .resetPose(Nodes.Robot.Start.RIGHT)
            .checkpoint(Nodes.Robot.Waypoint.MIDFIELD_RIGHT)
            .waypoint(Nodes.Robot.Start.RIGHT)
            .slow().endpoint(Nodes.Robot.Score.RIGHT).facing(Nodes.FieldElements.Hub.CENTER)
            .build();
    }
}
