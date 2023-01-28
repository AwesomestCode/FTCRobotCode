
package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

import static java.lang.Thread.sleep;

public class DetectAprilTagZoneUtil {
    static AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private static final double tagsize = 0.02;

    private static final double fx = 822.317;
    private static final double fy = 822.317;
    private static final double cx = 319.495;
    private static final double cy = 242.502;

    private static int numFramesWithoutDetection = 0;

    private static final float DECIMATION_HIGH = 1;
    private static final float DECIMATION_LOW = 1;
    private static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    private static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    private static int cameraMonitorViewId;
    private static OpenCvWebcam webcam;
    private static FtcDashboard dashboard;
    private static boolean initialised = false;

    private static int mostRecentDetection = -1;
    private static Thread detectionThread;
    public static void initialise(HardwareMap hardwareMap, Telemetry telemetry) {
        if(initialised) {
            return;
        }
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(webcam, 0);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        detectionThread = new Thread(() -> {
            // If there's been a new frame...
            while(true) {
                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

                /*
                 * Send some stats to the telemetry
                 */
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
                if(detections != null) {
                    // If we don't see any tags
                    if (detections.size() == 0) {
                        numFramesWithoutDetection++;

                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back
                        if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                        }
                    }
                    // We do see tags!
                    else {
                        numFramesWithoutDetection = 0;

                        // If the target is within 1 meter, turn on high decimation to
                        // increase the frame rate
                        if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                        }

                        for (AprilTagDetection detection : detections) {
                            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                            telemetry.addLine(String.format("Translation X: %.2f metres", detection.pose.x));
                            telemetry.addLine(String.format("Translation Y: %.2f metres", detection.pose.y));
                            telemetry.addLine(String.format("Translation Z: %.2f metres", detection.pose.z));
                            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                            if (detection.id == 114) {
                                mostRecentDetection = 1;
                            }
                            if (detection.id == 187) {
                                mostRecentDetection = 2;
                            }
                            if (detection.id == 196) {
                                mostRecentDetection = 3;
                            }
                        }
                    }
                }
                telemetry.update();
                try {
                    sleep(250);
                } catch (InterruptedException e) {
                    System.err.println("Detection Interrupted");
                    break;
                }
            }
        });

        detectionThread.start();

        initialised = true;
    }

    @SuppressWarnings("DuplicatedCode")
    public static int getZone(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
        if(!initialised) {
            initialise(hardwareMap, telemetry);
        }

        webcam.closeCameraDevice();
        detectionThread.interrupt();
        initialised = false;
        return mostRecentDetection;
    }
}
