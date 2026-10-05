//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto.policy;

import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.team1507.lib.core.auto.RouteNode;
import org.team1507.lib.core.swerve.SwerveAccelLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Drives the demo autos' routes with the Java policy in a simple 20 ms
 * simulation (the training env's physics: 1507Base's acceleration limiter,
 * then move at that velocity) and checks the robot reaches every node without
 * touching a field element. Catches a policy that matches Python number for
 * number but still can't drive our routes.
 */
class PolicyDrivesRoutesTest {

    private static final double DT = 0.02;
    private static final double PASS_RADIUS = 0.40;      // Accuracy.NORMAL
    private static final double NODE_TIME_LIMIT = 5.0;   // kAuto.MAX_SECONDS_PER_NODE
    private static final double MAX_SPEED = 5.04;

    private static PolicyRouteDriver driver;
    private static Json field;
    private static double bumper;

    @BeforeAll
    static void load() throws Exception {
        Json file = Json.read(PolicyRouteDriverTest.POLICY);
        driver = new PolicyRouteDriver(file);
        field = file.get("inputs").get("field");
        bumper = file.get("inputs").get("rays").get("subtract_m").asDouble();
    }

    @Test
    void throughTheHub() {
        drive("Through the Hub", 2.0, 4.1, 1.0, new double[][] {{6.3, 4.1}, {2.0, 4.1}});
    }

    @Test
    void aroundTheHub() {
        drive("Around the Hub", 3.5, 2.5, 1.0, new double[][] {{6.0, 4.035}, {3.5, 5.7}, {3.3, 2.8}});
    }

    @Test
    void midfieldAndBack() {
        drive("Midfield and Back", 3.5, 2.5, 0.8, new double[][] {{8.27, 2.0}, {3.5, 2.5}, {3.3, 2.8}});
    }

    private void drive(String name, double x, double y, double speedFraction, double[][] route) {
        SwerveAccelLimiter limiter = new SwerveAccelLimiter(
            new SwerveAccelLimiter.Limits(15.0, 10.0, Math.toRadians(1000), MAX_SPEED), DT);
        ChassisVelocities velocity = new ChassisVelocities(0, 0, 0);
        List<String> log = new ArrayList<>();
        double t = 0;
        for (int i = 0; i < route.length; i++) {
            RouteNode node = node(route[i], speedFraction);
            RouteNode next = node(route[Math.min(i + 1, route.length - 1)], speedFraction);
            double nodeStart = t;
            while (Math.hypot(route[i][0] - x, route[i][1] - y) > PASS_RADIUS) {
                if (t - nodeStart > NODE_TIME_LIMIT) {
                    fail(name + ": node " + i + " not reached in " + NODE_TIME_LIMIT + " s; robot at "
                        + String.format("(%.2f, %.2f)", x, y));
                }
                Translation2d command = driver.translation(new Pose2d(x, y, Rotation2d.ZERO), velocity, node, next);
                ChassisVelocities measured = velocity;
                velocity = limiter.limit(new ChassisVelocities(command.getX(), command.getY(), 0), () -> measured, t);
                x += velocity.vx * DT;
                y += velocity.vy * DT;
                t += DT;
                String hit = collision(x, y);
                if (hit != null) {
                    fail(name + ": hit " + hit + String.format(" at (%.2f, %.2f), %.2f s, driving to node %d", x, y, t, i));
                }
            }
            log.add(String.format("node %d at %.2f s", i, t));
        }
        System.out.println(name + ": " + String.join(", ", log));
    }

    private static RouteNode node(double[] p, double speedFraction) {
        return new RouteNode(RouteNode.Type.CHECKPOINT, new Pose2d(p[0], p[1], Rotation2d.ZERO), Rotation2d.ZERO,
            PASS_RADIUS, 30, false, MAX_SPEED * speedFraction, Double.POSITIVE_INFINITY, null);
    }

    /** The obstacle or wall the robot's square bumper overlaps, or null. */
    private static String collision(double x, double y) {
        for (Json obstacle : field.get("obstacles")) {
            double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
            for (Json c : obstacle.get("corners")) {
                minX = Math.min(minX, c.get(0).asDouble());
                maxX = Math.max(maxX, c.get(0).asDouble());
                minY = Math.min(minY, c.get(1).asDouble());
                maxY = Math.max(maxY, c.get(1).asDouble());
            }
            if (x > minX - bumper && x < maxX + bumper && y > minY - bumper && y < maxY + bumper) {
                return obstacle.get("name").asText();
            }
        }
        Json walls = field.get("walls").get("corners");
        double length = walls.get(2).get(0).asDouble();
        double width = walls.get(2).get(1).asDouble();
        if (x < bumper || y < bumper || x > length - bumper || y > width - bumper) {
            return "the field wall";
        }
        return null;
    }
}
