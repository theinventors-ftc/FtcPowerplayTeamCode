package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.inventors.ftc.robotbase.GamepadExEx;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "AprilTag Aligning", group = "Tests")
public class ApriltagAligning extends CommandOpMode {
    private GamepadExEx driverOp;
    public final int LEFT = 4;
    public final int MIDDLE = 5;
    public final int RIGHT = 6;
    private static boolean active_align = true;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private double forwardSpeed = 0, strafeSpeed = 0;
    private PositionControllerSubsystem positionControllerSubsystem;
    private HeadingControllerSubsystem headingControllerSubsystem;

    private static double tagTargetX = 9, tagTargetY = 33, tagTargetRX = 15; // X is yaw
    public double tagErrorX = 0, tagErrorY = 0, tagErrorRX = 0;
    public double tagErrorX_filt = 0, tagErrorY_filt = 0, tagErrorRX_filt = 0;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Boolean tagFound = false;
    private List<AprilTagDetection> currentDetections = new ArrayList<>();
    private AprilTagDetection tagOfInterest;

    private MultipleTelemetry doubleTelemetry;

    // Configuration Params
    public boolean drawAxes = false;
    public boolean drawCubeProjection = false;
    public boolean drawTagOutline = false;
    public double fx = 578.272;
    public double fy = 578.272;
    public double cx = 402.145;
    public double cy = 221.506;
    public static int decimation = 2;

    public int viewportSizeX = 640;
    public int viewportSizeY = 480;

    public static double KpX = 0.036, KpY = 0.042, KpRX = 0.03;

    public static double KiX = 0.03, KiY = 0.03, KiRX = 0.03;
    public static double KdX = 0, KdY = 0, KdRX = 0;

    public static double alpha = 0.2;
    public static double integralWorkingBounds = 100;
    public static double integralClippingBounds = 1;

    public static double FILTER_CONSTANT = 0.8;

    private ElapsedTime time;
    private long startTime = 0;

    public static int camMillis = 1;
    public static int camGain = 250;

    private void initDetectionStructure() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(drawAxes)
                .setDrawCubeProjection(drawCubeProjection)
                .setDrawTagOutline(drawTagOutline)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(decimation);

//        streamingProcessor = new CameraStreamProcessor();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");

        VisionPortal.Builder builder = new VisionPortal.Builder()
//                .addProcessor(streamingProcessor)
                .addProcessor(aprilTag)
                .setCamera(webcam)
                .setCameraResolution(new Size(viewportSizeX, viewportSizeY))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .setAutoStopLiveView(true);

        visionPortal = builder.build();

        setManualExposure(visionPortal, 4, 250);

//        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) visionPortal, 0);
    }

    private void setManualExposure(VisionPortal portal, int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (portal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            doubleTelemetry.addData("Camera", "Waiting");
            doubleTelemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            doubleTelemetry.addData("Camera", "Ready");
            doubleTelemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public boolean aprilTagCheck() {
        this.currentDetections = aprilTag.getDetections();

//        telemetryAprilTag();

        if (currentDetections.size() != 0) {
            tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if(tag.id == MIDDLE) {
                    tagFound = true;

                    doubleTelemetry.addData("Found", "ID %d (%s)", tag.id, tag.metadata.name);
                    doubleTelemetry.addData("Range",  "%5.1f inches", tag.ftcPose.range);
                    doubleTelemetry.addData("Bearing","%3.0f degrees", tag.ftcPose.bearing);
                    doubleTelemetry.addData("Yaw","%3.0f degrees", tag.ftcPose.yaw);

                    if (tag.metadata != null) {
                        tagErrorX = tagTargetX - tag.ftcPose.yaw;
                        tagErrorY = tagTargetY - tag.ftcPose.range;
                        tagErrorRX = tagTargetRX - tag.ftcPose.bearing;
                    }
                }
            }
        }
        return tagFound;
    }

    @Override
    public void initialize() {
        doubleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.driverOp = new GamepadExEx(gamepad1);
        positionControllerSubsystem = new PositionControllerSubsystem(() -> tagErrorX_filt, () -> tagErrorY_filt, () -> tagErrorRX_filt,
                KpX, KpY, KpRX,  KiX, KiY, KiRX, KdX, KdY, KdRX, alpha, integralWorkingBounds, integralClippingBounds); // 0.016, 0.036

        driveSubsystem = new DriveSubsystem(hardwareMap);
//        driveCommand = new DriveCommand(driveSubsystem, () -> forward(),
//                () -> strafe(), () -> driverOp.getRightX());

        CommandScheduler.getInstance().registerSubsystem(driveSubsystem);
//        driveSubsystem.setDefaultCommand(driveCommand);

        initDetectionStructure();

        time = new ElapsedTime();
        time.reset();
    }

    @Override
    public void run() {
        startTime = time.now(TimeUnit.MICROSECONDS);
        aprilTagCheck();

//        tagErrorX_filt = (1-FILTER_CONSTANT)*tagErrorX + FILTER_CONSTANT * tagErrorX_filt;
//        tagErrorY_filt = (1-FILTER_CONSTANT)*tagErrorY + FILTER_CONSTANT * tagErrorY_filt;
//        tagErrorRX_filt = (1-FILTER_CONSTANT)*tagErrorRX + FILTER_CONSTANT * tagErrorRX_filt;
        tagErrorX_filt = tagErrorX;
        tagErrorY_filt = tagErrorY;
        tagErrorRX_filt = tagErrorRX;

        driveSubsystem.drive(strafe(), forward(), rotation());

        doubleTelemetry.addData("ErrorX(raw): ", "%5.1f", tagErrorX);
        doubleTelemetry.addData("ErrorY(raw): ", "%5.1f", tagErrorY);
        doubleTelemetry.addData("ErrorRX(raw):", " %5.1f", tagErrorRX);
        doubleTelemetry.addData("ErrorX(filtered): ", "%5.1f", tagErrorX_filt);
        doubleTelemetry.addData("ErrorY(filtered): ", "%5.1f", tagErrorY_filt);
        doubleTelemetry.addData("ErrorRX(filtered):", " %5.1f", tagErrorRX_filt);

        active_align = driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER);

        if (active_align) telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", forward(), strafe(), rotation());
        else telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", forward(), strafe(), rotation());

        doubleTelemetry.addData("FPS:", visionPortal.getFps());
        doubleTelemetry.addData("Hz:", 1000000.0 / (time.now(TimeUnit.MICROSECONDS)-startTime) );
        doubleTelemetry.update();
    }

    public double strafe() {
        if(active_align) {
            return positionControllerSubsystem.calculateStrafeSpeed();
        } else {
            return driverOp.getLeftX() * 0.7;
        }
    }
    public double forward() {
        if(active_align) {
            return positionControllerSubsystem.calculateForwardSpeed();
        } else {
            return driverOp.getLeftY() * 0.7;
        }
    }

    public double rotation() {
        if(active_align) {
            return positionControllerSubsystem.calculateRotateSpeed();
        } else {
            return driverOp.getRightX()* 0.5;
        }
    }
}
