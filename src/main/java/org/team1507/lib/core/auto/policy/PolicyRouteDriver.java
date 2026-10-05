//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto.policy;

import java.io.IOException;
import java.nio.file.Path;
import org.team1507.lib.core.auto.RouteDriver;
import org.team1507.lib.core.auto.RouteNode;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

/**
 * The policy driver: the trained network from Swerve-Policy-Playground decides
 * how the robot moves toward each node, steering around field elements itself.
 * Used by autos written with {@code new AutoSequence(Driver.POLICY)}.
 *
 * <p>Each loop: build the 16 inputs, run the network, and drive at the
 * field-relative velocity it outputs (capped at the node's speed). The route
 * runner does everything else, the same as for the classic driver: turning,
 * when a node counts as reached, the precise finish at endpoints, giving up.
 *
 * <p>Logs to {@code Auto/Policy}.
 */
public final class PolicyRouteDriver implements RouteDriver {

    private static final TelemetryTable TABLE = Telemetry.getTable("Auto/Policy");

    private final PolicyNetwork network;
    private final PolicyInputs inputs;
    private final double outputScale;
    private final String source;

    /** Builds the driver from a parsed policy file. */
    public PolicyRouteDriver(Json policy) {
        network = new PolicyNetwork(policy.get("network"));
        inputs = new PolicyInputs(policy.get("inputs"));
        outputScale = policy.get("outputs").get("scale_mps").asDouble();
        source = policy.has("source") && policy.get("source").has("checkpoint")
            ? policy.get("source").get("checkpoint").asText() : "unknown";
        if (network.inputCount() != inputs.count()) {
            throw new IllegalArgumentException("network takes " + network.inputCount()
                + " inputs, the file describes " + inputs.count());
        }
    }

    /** Reads a policy file (the JSON written by the playground's export). */
    public static PolicyRouteDriver load(Path file) throws IOException {
        return new PolicyRouteDriver(Json.read(file));
    }

    /**
     * Reads a policy file, or reports the problem on the Driver Station and
     * returns null (policy autos then refuse to build; classic autos still work).
     */
    public static PolicyRouteDriver loadOrNull(Path file) {
        try {
            PolicyRouteDriver driver = load(file);
            TABLE.log("Source", driver.source);
            return driver;
        } catch (IOException | RuntimeException e) {
            DriverStationErrors.reportError("Policy driver not loaded from " + file + ": " + e.getMessage(), false);
            return null;
        }
    }

    /** The parts, for tests. */
    PolicyNetwork network() {
        return network;
    }

    PolicyInputs inputs() {
        return inputs;
    }

    @Override
    public Translation2d translation(Pose2d pose, ChassisVelocities fieldSpeed,
                                     RouteNode current, RouteNode next) {
        long start = System.nanoTime();
        Translation2d node = current.target().getTranslation();
        Translation2d after = next.target().getTranslation();
        double[] in = inputs.build(pose.getX(), pose.getY(), fieldSpeed.vx, fieldSpeed.vy,
                                   node.getX(), node.getY(), after.getX(), after.getY());
        double[] action = network.run(in);

        Translation2d velocity = new Translation2d(action[0] * outputScale, action[1] * outputScale);
        double speed = velocity.getNorm();
        if (speed > current.speed()) {
            velocity = velocity.times(current.speed() / speed);
        }

        TABLE.log("Inputs", in);
        TABLE.log("Action", action);
        TABLE.log("CommandX", velocity.getX());
        TABLE.log("CommandY", velocity.getY());
        TABLE.log("RunMs", (System.nanoTime() - start) / 1e6);
        return velocity;
    }
}
