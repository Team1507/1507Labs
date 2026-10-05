//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto.policy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Java must drive exactly like Python. The exported policy file carries test
 * cases recorded from the training env; for each one, Java's inputs and
 * Java's network output must match what Python computed.
 */
class PolicyRouteDriverTest {

    static final Path POLICY = Path.of("src/main/deploy/policy/randomizer_policy.json");

    private static Json file;
    private static PolicyRouteDriver driver;

    @BeforeAll
    static void load() throws Exception {
        file = Json.read(POLICY);
        driver = new PolicyRouteDriver(file);
    }

    @Test
    void inputsMatchPython() {
        Json tests = file.get("tests");
        double tolerance = tests.get("input_tolerance").asDouble();
        List<String> problems = new ArrayList<>();
        int n = 0;
        for (Json c : tests.get("cases")) {
            Json robot = c.get("robot");
            double[] node = c.get("node").asDoubles();
            double[] next = c.get("next_node").asDoubles();
            double[] expected = c.get("inputs").asDoubles();
            double[] actual = driver.inputs().build(
                robot.get("x").asDouble(), robot.get("y").asDouble(),
                robot.get("vx").asDouble(), robot.get("vy").asDouble(),
                node[0], node[1], next[0], next[1]);
            compare("case " + n + " input", expected, actual, tolerance, problems);
            n++;
        }
        assertTrue(n > 0, "the policy file has no test cases");
        if (!problems.isEmpty()) {
            fail(problems.size() + " input mismatches, first: " + problems.subList(0, Math.min(5, problems.size())));
        }
    }

    @Test
    void networkMatchesPython() {
        Json tests = file.get("tests");
        double tolerance = tests.get("action_tolerance").asDouble();
        List<String> problems = new ArrayList<>();
        int n = 0;
        for (Json c : tests.get("cases")) {
            double[] expected = c.get("action").asDoubles();
            double[] actual = driver.network().run(c.get("inputs").asDoubles());
            compare("case " + n + " action", expected, actual, tolerance, problems);
            n++;
        }
        if (!problems.isEmpty()) {
            fail(problems.size() + " action mismatches, first: " + problems.subList(0, Math.min(5, problems.size())));
        }
    }

    @Test
    void networkIsFastEnoughForA20msLoop() {
        double[] in = file.get("tests").get("cases").get(0).get("inputs").asDoubles();
        for (int i = 0; i < 2000; i++) {
            driver.network().run(in);   // warm up the JIT
        }
        long start = System.nanoTime();
        int runs = 1000;
        for (int i = 0; i < runs; i++) {
            driver.network().run(in);
        }
        double ms = (System.nanoTime() - start) / 1e6 / runs;
        assertTrue(ms < 2.0, "one network run took " + ms + " ms");
    }

    @Test
    void readsJson() {
        Json j = Json.parse("{\"a\": [1, -2.5e-1, \"x\\\"y\"], \"b\": {\"c\": true, \"d\": null}}");
        assertEquals(-0.25, j.get("a").get(1).asDouble());
        assertEquals("x\"y", j.get("a").get(2).asText());
        assertTrue(j.get("b").has("d"));
    }

    private static void compare(String what, double[] expected, double[] actual, double tolerance,
                                List<String> problems) {
        if (expected.length != actual.length) {
            problems.add(what + ": length " + actual.length + ", expected " + expected.length);
            return;
        }
        for (int i = 0; i < expected.length; i++) {
            if (Math.abs(expected[i] - actual[i]) > tolerance) {
                problems.add(String.format("%s[%d]: %.6f, expected %.6f", what, i, actual[i], expected[i]));
            }
        }
    }
}
