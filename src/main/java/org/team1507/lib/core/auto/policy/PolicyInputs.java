//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto.policy;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds the policy's 16 inputs from the robot's state, exactly as the
 * training env did ({@code path_randomizer/swerve_env.py _get_obs}). Every
 * scale, and the obstacles the rays are cast against, come from the
 * {@code inputs} section of the exported policy file, so a retrained policy
 * with different settings needs no Java change.
 *
 * <pre>
 *   0-1   velocity / scale            (clamped -1..1)
 *   2-3   position / field size       (clamped 0..1)
 *   4-5   vector to the current node / scale, shrunk to length 1 if longer
 *   6-7   vector to the next node, the same way
 *   8-15  rays: free distance in 8 field-relative directions, 0 (touching) .. 1 (clear)
 * </pre>
 */
public final class PolicyInputs {

    /** The input order this class builds; the file's {@code inputs.order} must match. */
    static final List<String> ORDER = List.of(
        "vx_n", "vy_n", "rx_n", "ry_n", "dx0_n", "dy0_n", "dx1_n", "dy1_n",
        "ray0", "ray1", "ray2", "ray3", "ray4", "ray5", "ray6", "ray7");

    private final double velocityScale;
    private final double fieldLength;
    private final double fieldWidth;
    private final double nodeScale;
    private final double nodeMaxLength;
    private final double[] rayCos;
    private final double[] raySin;
    private final double rayRange;
    private final double raySubtract;
    private final double[][] segments;   // each {ax, ay, bx, by}: obstacle edges and walls

    /** Reads the {@code inputs} section of the policy file. */
    public PolicyInputs(Json inputs) {
        List<String> order = new ArrayList<>();
        inputs.get("order").forEach(n -> order.add(n.asText()));
        if (!order.equals(ORDER)) {
            throw new IllegalArgumentException("policy inputs are " + order + ", but this code builds " + ORDER);
        }
        velocityScale = inputs.get("velocity").get("scale_mps").asDouble();
        fieldLength = inputs.get("position").get("field_length_m").asDouble();
        fieldWidth = inputs.get("position").get("field_width_m").asDouble();
        nodeScale = inputs.get("node_vectors").get("scale_m").asDouble();
        nodeMaxLength = inputs.get("node_vectors").get("max_length").asDouble();

        Json rays = inputs.get("rays");
        int count = rays.get("count").asInt();
        rayCos = new double[count];
        raySin = new double[count];
        for (int k = 0; k < count; k++) {
            double angle = Math.toRadians(rays.get("first_angle_deg").asDouble() + k * rays.get("step_deg").asDouble());
            rayCos[k] = Math.cos(angle);
            raySin[k] = Math.sin(angle);
        }
        rayRange = rays.get("range_m").asDouble();
        raySubtract = rays.get("subtract_m").asDouble();

        List<double[]> edges = new ArrayList<>();
        Json field = inputs.get("field");
        for (Json obstacle : field.get("obstacles")) {
            addEdges(edges, obstacle.get("corners"));
        }
        addEdges(edges, field.get("walls").get("corners"));
        segments = edges.toArray(new double[0][]);
    }

    private static void addEdges(List<double[]> edges, Json corners) {
        int n = corners.size();
        for (int i = 0; i < n; i++) {
            Json a = corners.get(i);
            Json b = corners.get((i + 1) % n);
            edges.add(new double[] {a.get(0).asDouble(), a.get(1).asDouble(), b.get(0).asDouble(), b.get(1).asDouble()});
        }
    }

    /** How many inputs this builds. */
    public int count() {
        return 8 + rayCos.length;
    }

    /**
     * The inputs for one loop. Everything is field-relative, in meters and m/s.
     *
     * @param x, y         robot position
     * @param vx, vy       robot measured velocity
     * @param nodeX, nodeY the node being driven to
     * @param nextX, nextY the node after it (the same node at the end of the route)
     */
    public double[] build(double x, double y, double vx, double vy,
                          double nodeX, double nodeY, double nextX, double nextY) {
        double[] in = new double[count()];
        in[0] = clamp(vx / velocityScale, -1, 1);
        in[1] = clamp(vy / velocityScale, -1, 1);
        in[2] = clamp(x / fieldLength, 0, 1);
        in[3] = clamp(y / fieldWidth, 0, 1);
        nodeVector(in, 4, nodeX - x, nodeY - y);
        nodeVector(in, 6, nextX - x, nextY - y);
        for (int k = 0; k < rayCos.length; k++) {
            double d = rayDistance(x, y, rayCos[k], raySin[k]);
            in[8 + k] = clamp(d - raySubtract, 0, rayRange) / rayRange;
        }
        return in;
    }

    private void nodeVector(double[] in, int at, double dx, double dy) {
        dx /= nodeScale;
        dy /= nodeScale;
        double length = Math.hypot(dx, dy);
        if (length > nodeMaxLength) {
            dx = dx / length * nodeMaxLength;
            dy = dy / length * nodeMaxLength;
        }
        in[at] = dx;
        in[at + 1] = dy;
    }

    /**
     * Distance from (x, y) along direction (dx, dy) to the nearest edge, or
     * range + subtract when nothing is hit (the same as the training code).
     */
    private double rayDistance(double x, double y, double dx, double dy) {
        double nearest = Double.POSITIVE_INFINITY;
        for (double[] s : segments) {
            double ex = s[2] - s[0];
            double ey = s[3] - s[1];
            double wx = s[0] - x;
            double wy = s[1] - y;
            double denom = dx * ey - dy * ex;
            if (Math.abs(denom) <= 1e-12) {
                continue;   // parallel
            }
            double t = (wx * ey - wy * ex) / denom;   // along the ray
            double u = (wx * dy - wy * dx) / denom;   // along the edge
            if (t >= 0 && u >= 0 && u <= 1 && t < nearest) {
                nearest = t;
            }
        }
        return Double.isInfinite(nearest) ? rayRange + raySubtract : nearest;
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }
}
