package org.firstinspires.ftc.teamcode.PowerPlayRobot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class ApriltagDetectionSubsystem {
    public static final int LEFT = 483;
    public static final int MIDDLE = 484;
    public static final int RIGHT = 485;

    private Telemetry telemetry, dashTelemetry;
    private HardwareMap hardwareMap;
    private String webcamName;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double fx = 578.272;
    private double fy = 578.272;
    private double cx = 402.145;
    private double cy = 221.506;

    private final DistanceUnit distanceUnit;
    private final AngleUnit angleUnit;

    private Boolean tagFound = false;
    private List<AprilTagDetection> currentDetections = new ArrayList<>();

    private AprilTagDetection tagOfInterest;

    public ApriltagDetectionSubsystem(Telemetry telemetry, Telemetry dashTelemetry, HardwareMap hm, String cameraName, DistanceUnit distanceUnit,
                                      AngleUnit angleUnit) {
        this.telemetry = telemetry;
        this.dashTelemetry = dashTelemetry;

        this.hardwareMap = hm;
        this.webcamName = cameraName;

        this.distanceUnit = distanceUnit;
        this.angleUnit = angleUnit;

        initDetectionStructure();
    }

    public ApriltagDetectionSubsystem(Telemetry telemetry, Telemetry dashTelemetry, HardwareMap hm, String cameraName) {
        this.telemetry = telemetry;
        this.dashTelemetry = dashTelemetry;

        this.hardwareMap = hm;
        this.webcamName = cameraName;

        this.distanceUnit = DistanceUnit.CM;
        this.angleUnit = AngleUnit.DEGREES;

        initDetectionStructure();
    }

    private void initDetectionStructure() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(this.distanceUnit, this.angleUnit)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, webcamName));

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    public boolean aprilTagCheck() {
        this.currentDetections = aprilTag.getDetections();

        telemetryAprilTag();

        if (currentDetections.size() != 0) {
            tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                }
            }
        }
        return tagFound;
    }

    public AprilTagDetection getTagOfInterest() {
        return tagOfInterest;
    }

    private void telemetryAprilTag() {
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    public DistanceUnit getDistanceUnit() {
        return distanceUnit;
    }

    public AngleUnit getAngleUnit() {
        return angleUnit;
    }
}
