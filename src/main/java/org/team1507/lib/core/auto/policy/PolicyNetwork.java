//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto.policy;


/**
 * The trained policy network, run in plain Java: fully connected layers,
 * {@code y = activation(weights × x + biases)}. Loaded from the {@code network}
 * section of the exported policy file (Swerve-Policy-Playground's export).
 *
 * <p>The path policy is 16 → 256 → 256 → 3, about 68,000 multiply-adds per
 * call: well under a millisecond.
 */
public final class PolicyNetwork {

    private final double[][][] weights;   // [layer][out][in]
    private final double[][] biases;      // [layer][out]
    private final boolean[] tanh;         // per layer: tanh if true, else relu

    /** Reads the {@code network} section: {@code layers[]} with weights[out][in], biases[out], activation. */
    public PolicyNetwork(Json network) {
        Json layers = network.get("layers");
        int count = layers.size();
        weights = new double[count][][];
        biases = new double[count][];
        tanh = new boolean[count];
        for (int l = 0; l < count; l++) {
            Json layer = layers.get(l);
            int in = layer.get("inputs").asInt();
            int out = layer.get("outputs").asInt();
            String activation = layer.get("activation").asText();
            if (!activation.equals("relu") && !activation.equals("tanh")) {
                throw new IllegalArgumentException("layer " + l + ": unknown activation '" + activation + "'");
            }
            tanh[l] = activation.equals("tanh");
            weights[l] = new double[out][in];
            biases[l] = new double[out];
            for (int o = 0; o < out; o++) {
                Json row = layer.get("weights").get(o);
                for (int i = 0; i < in; i++) {
                    weights[l][o][i] = row.get(i).asDouble();
                }
                biases[l][o] = layer.get("biases").get(o).asDouble();
            }
            if (l > 0 && in != weights[l - 1].length) {
                throw new IllegalArgumentException("layer " + l + " takes " + in
                    + " inputs but layer " + (l - 1) + " gives " + weights[l - 1].length);
            }
        }
    }

    /** How many inputs the network takes. */
    public int inputCount() {
        return weights[0][0].length;
    }

    /** Runs the network. The result is the action, each value in -1..1. */
    public double[] run(double[] input) {
        if (input.length != inputCount()) {
            throw new IllegalArgumentException("expected " + inputCount() + " inputs, got " + input.length);
        }
        double[] x = input;
        for (int l = 0; l < weights.length; l++) {
            double[] y = new double[weights[l].length];
            for (int o = 0; o < y.length; o++) {
                double sum = biases[l][o];
                double[] row = weights[l][o];
                for (int i = 0; i < row.length; i++) {
                    sum += row[i] * x[i];
                }
                y[o] = tanh[l] ? Math.tanh(sum) : Math.max(0.0, sum);
            }
            x = y;
        }
        return x;
    }
}
