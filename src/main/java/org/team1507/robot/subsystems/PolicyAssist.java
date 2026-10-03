package org.team1507.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import org.team1507.lib.core.framework.Subsystem1507;

// ─────────────────────────────────────────────────────────────────────────────
// PolicyAssist
//
// NetworkTables bridge between the Java robot code and the Python teleop-assist
// policy running as a separate process (sim: on the same PC; comp: on SystemCore).
//
// Java publishes raw joystick + robot state → Python reads, runs SAC inference,
// writes back ChassisSpeeds → Java reads and feeds to swerve.drive().
//
// If the Python bridge heartbeat goes stale (>200 ms), isActive() returns false
// and Robot.java falls back to the raw joystick path automatically.
//
// NT topic layout:
//   policy/input/joy_left_y, joy_left_x, joy_right_x   (field-frame, deadbanded)
//   policy/input/heading                                 (radians)
//   policy/input/vx, vy, omega                          (robot-relative m/s, rad/s)
//   policy/input/rx, ry                                 (field position, meters)
//   policy/output/vx, vy, omega                         (robot-relative, m/s & rad/s)
//   policy/heartbeat                                     (FPGA timestamp, doubles)
//   policy/enabled                                       (boolean publisher → SmartDashboard)
// ─────────────────────────────────────────────────────────────────────────────
public final class PolicyAssist extends Subsystem1507 {

    // ── NT publishers (Java → Python) ─────────────────────────────────────────
    private final DoublePublisher pubJoyLeftY;
    private final DoublePublisher pubJoyLeftX;
    private final DoublePublisher pubJoyRightX;
    private final DoublePublisher pubHeading;
    private final DoublePublisher pubVx;
    private final DoublePublisher pubVy;
    private final DoublePublisher pubOmega;
    private final DoublePublisher pubRx;
    private final DoublePublisher pubRy;

    // ── NT subscribers (Python → Java) ────────────────────────────────────────
    private final DoubleSubscriber subOutVx;
    private final DoubleSubscriber subOutVy;
    private final DoubleSubscriber subOutOmega;
    private final DoubleSubscriber subHeartbeat;

    // ── State ──────────────────────────────────────────────────────────────────
    private boolean enabled = false;
    private double lastHeartbeat = 0.0;

    // ── Stale timeout ──────────────────────────────────────────────────────────
    private static final double HEARTBEAT_TIMEOUT_S = 0.2;

    public PolicyAssist() {
        super("PolicyAssist");

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable input  = inst.getTable("policy").getSubTable("input");
        NetworkTable output = inst.getTable("policy").getSubTable("output");

        pubJoyLeftY  = input.getDoubleTopic("joy_left_y").publish();
        pubJoyLeftX  = input.getDoubleTopic("joy_left_x").publish();
        pubJoyRightX = input.getDoubleTopic("joy_right_x").publish();
        pubHeading   = input.getDoubleTopic("heading").publish();
        pubVx        = input.getDoubleTopic("vx").publish();
        pubVy        = input.getDoubleTopic("vy").publish();
        pubOmega     = input.getDoubleTopic("omega").publish();
        pubRx        = input.getDoubleTopic("rx").publish();
        pubRy        = input.getDoubleTopic("ry").publish();

        subOutVx      = output.getDoubleTopic("vx").subscribe(0.0);
        subOutVy      = output.getDoubleTopic("vy").subscribe(0.0);
        subOutOmega   = output.getDoubleTopic("omega").subscribe(0.0);
        subHeartbeat  = inst.getTable("policy").getDoubleTopic("heartbeat").subscribe(0.0);
    }

    // ── Called from Robot.java every loop before reading getPolicySpeeds() ────

    /**
     * Publishes the current robot state to NT for the Python bridge to consume.
     *
     * @param joyLeftY   deadbanded forward stick (field-frame, +forward)
     * @param joyLeftX   deadbanded strafe stick  (field-frame, +left)
     * @param joyRightX  deadbanded rotation stick (+CCW)
     * @param pose       current robot field pose
     * @param speeds     current robot-relative ChassisSpeeds
     * @param headingRad current heading in radians
     */
    public void update(
        double joyLeftY, double joyLeftX, double joyRightX,
        Pose2d pose, ChassisSpeeds speeds, double headingRad
    ) {
        pubJoyLeftY .set(joyLeftY);
        pubJoyLeftX .set(joyLeftX);
        pubJoyRightX.set(joyRightX);
        pubHeading  .set(headingRad);
        pubVx       .set(speeds.vxMetersPerSecond);
        pubVy       .set(speeds.vyMetersPerSecond);
        pubOmega    .set(speeds.omegaRadiansPerSecond);
        pubRx       .set(pose.getX());
        pubRy       .set(pose.getY());

        lastHeartbeat = subHeartbeat.get();

        log("Enabled", enabled);
        log("BridgeActive", isActive());
        log("RawVx", speeds.vxMetersPerSecond);
        log("RawVy", speeds.vyMetersPerSecond);
    }

    /**
     * Returns robot-relative ChassisSpeeds computed by the policy.
     * Call only after {@link #isActive()} returns true.
     */
    public ChassisSpeeds getPolicySpeeds() {
        double vx    = subOutVx.get();
        double vy    = subOutVy.get();
        double omega = subOutOmega.get();

        log("PolicyVx",    vx);
        log("PolicyVy",    vy);
        log("PolicyOmega", omega);

        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * Returns true when the policy is enabled by the driver AND the Python bridge
     * is alive (heartbeat received within the last 200 ms).
     */
    public boolean isActive() {
        if (!enabled) return false;
        double age = Timer.getFPGATimestamp() - lastHeartbeat;
        return age < HEARTBEAT_TIMEOUT_S;
    }

    public boolean isEnabled() { return enabled; }

    public void setEnabled(boolean on) {
        enabled = on;
        if (on) {
            warn("Policy assist enabled — right bumper to disable.");
        } else {
            warn("Policy assist disabled — raw joystick active.");
        }
    }
}
