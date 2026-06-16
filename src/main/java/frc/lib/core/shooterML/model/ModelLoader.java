//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.shooterML.model;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;

import com.fasterxml.jackson.databind.JsonNode;

public class ModelLoader {

    public static ShooterModel load(String path, Supplier<Pose2d> poseSupplier) {
        try {
            String json = Files.readString(Path.of(path));

            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(json);

            ModelConfig config = new ModelConfig();
            config.a = root.get("a").asDouble();
            config.b = root.get("b").asDouble();
            config.c = root.get("c").asDouble();
            config.d = root.get("d").asDouble();
            config.e = root.get("e").asDouble();
            config.f = root.get("f").asDouble();
            config.g = root.get("g").asDouble();

            return new QuadraticShooterModel(config);

        } catch (Exception ex) {
            ex.printStackTrace();
            return new SimpleInterpolationModel();
        }
    }
}
