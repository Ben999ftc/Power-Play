package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class LeftBlue extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int one = 1;
    int two = 2;
    int three = 3;

    AprilTagDetection tagOfInterest = null;

    SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
    Lift lift = new Lift();

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        Pose2d startPose = new Pose2d(41, 65, Math.toRadians(270));
        Vector2d parkPose = new Vector2d(36, 12);
        robot.setPoseEstimate(startPose);
        TrajectorySequence preload = robot.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    robot.claw.setPosition(0.5);
                })
                .splineToConstantHeading(new Vector2d(36, 50), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(36, 12), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    lift.setHeight(380);
                    lift.armAngle(190);
                })
                .turn(Math.toRadians(135))
                .addDisplacementMarker(() -> {
                    robot.vee.setPosition(1);
                })
                .back(8)
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    robot.claw.setPosition(0.25);
                })
                .forward(8)
                .addDisplacementMarker(() -> {
                    lift.setHeight(0);
                    robot.vee.setPosition(0.6);
                    lift.armAngle(0);
                })
                .turn(Math.toRadians(-45))
                .build();
        TrajectorySequence cycle = robot.trajectorySequenceBuilder(preload.end())
                // Raise slides to correct height, put arm out slightly
                .addDisplacementMarker(() -> {
                    lift.setHeight(1);
                    lift.armAngle(3);
                    robot.claw.setPosition(0.1);
                })
                .splineToConstantHeading(new Vector2d(65, 12), Math.toRadians(0))
                .waitSeconds(3)
                // Claw grips, slides raise
                .addDisplacementMarker(() -> {
                    robot.claw.setPosition(0.5);
                    lift.setHeight(lift.getHeight() + 125);
                })
                .splineToConstantHeading(new Vector2d(36, 12), Math.toRadians(180))
                // Raise slides, raise arm
                .addDisplacementMarker(() -> {
                    lift.setHeight(380);
                    lift.armAngle(190);
                })
                .turn(Math.toRadians(45))
                // Lower V
                .addDisplacementMarker(() -> {
                    robot.vee.setPosition(1);
                })
                .back(8)
                // Claw releases
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    robot.claw.setPosition(0.25);
                })
                .forward(8)
                // Lower slides, lower arm, raise V
                .addDisplacementMarker(() -> {
                    lift.setHeight(0);
                    robot.vee.setPosition(0.6);
                    lift.armAngle(0);
                })
                .turn(Math.toRadians(-45))
                .build();
        TrajectorySequence park = robot.trajectorySequenceBuilder(cycle.end())
                .splineToConstantHeading(parkPose, Math.toRadians(180))
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == one || tag.id == two || tag.id == three)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
            }
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest.id == one){
            parkPose = new Vector2d(60, 12);
        }
        else if (tagOfInterest.id == three){
            parkPose = new Vector2d(12, 12);
        }
        robot.followTrajectorySequence(preload);
        robot.followTrajectorySequence(cycle);
        robot.followTrajectorySequence(park);
    }

}
