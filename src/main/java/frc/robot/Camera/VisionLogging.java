package frc.robot.Camera;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import frc.robot.Camera.BaseCam.AprilTagResult;

public class VisionLogging {
    private static List<VisionLogEntry> visionEntries = new ArrayList<VisionLogEntry>();

    public static class VisionLogEntry {
        public double timestamp;
        public double visionX;
        public double visionY;
        public double visionTheta;
        public double odomX;
        public double odomY;
        public double odomTheta;
        public double distToTag;
        public int tagCount;
        public double ambiguity;

        public VisionLogEntry(AprilTagResult result) {
            timestamp = result.time;
            visionX = result.pose.getX();
            visionY = result.pose.getY();
            visionTheta = result.pose.getRotation().getDegrees();
            distToTag = result.distToTag;
            tagCount = result.tagCount;
            ambiguity = result.ambiguity;
        }
    }

    public static void addVisionEntry(AprilTagResult result, double odomX, double odomY, double odomTheta) {
        VisionLogEntry entry = new VisionLogEntry(result);
        entry.odomX = odomX;
        entry.odomY = odomY;
        entry.odomTheta = odomTheta;
        visionEntries.add(entry);
    }

    public static void dumpLogs() {
        try {
            FileWriter writer = new FileWriter("/U/visionLogs/vision_logs.csv");
            writer.write("Time,VisionX,VisionY,VisionTheta,OdomX,OdomY,OdomTheta,DistToTag,TagCount,Ambiguity\n");
            for (VisionLogEntry entry : visionEntries) {
                writer.write(String.format(
                        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f\n",
                        entry.timestamp,
                        entry.visionX,
                        entry.visionY,
                        entry.visionTheta,
                        entry.odomX,
                        entry.odomY,
                        entry.odomTheta,
                        entry.distToTag,
                        entry.tagCount,
                        entry.ambiguity));
            }
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
    }

    public static void setUpDirectory() {
        try {
            File dir = new File("/U/visionLogs");
            if (!dir.exists()) {
                dir.mkdir();
            }
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
    }

}
