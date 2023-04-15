package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous
public class LeftHigh extends LinearOpMode {
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

    int stack_height = 4;
    int[] heights = {0, 45, 80, 95, 140};

    AprilTagDetection tagOfInterest = null;

    boolean intake = true;



    @Override
    public void runOpMode()
    {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift();
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

        Pose2d startPose = new Pose2d(31.1, 61.6, Math.toRadians(90));
        Pose2d dropPose = new Pose2d(31.8, 3.7, Math.toRadians(45));
        int position = 2;
        robot.setPoseEstimate(startPose);
        telemetry.addData( "Passed", "1");
        telemetry.update();

        TrajectorySequence preload = robot.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.backArmSensor();
                    lift.setHeight(390);
                })
                .lineToConstantHeading(new Vector2d(36, 50))
                .splineToConstantHeading(new Vector2d(36, 25), Math.toRadians(270))
                .splineToSplineHeading(dropPose, Math.toRadians(225))
                .addTemporalMarker(() -> {
                    lift.openClaw();
                    lift.armAngle(0);
                })
                .waitSeconds(0.2)
                .build();
        TrajectorySequence cycle = robot.trajectorySequenceBuilder(preload.end())
                .splineToSplineHeading(new Pose2d(60, 9.6, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(66, 9.6), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    lift.closeClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    lift.setHeight(390);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.backArmSensor();
                })
                .lineToConstantHeading(new Vector2d(58, 11.5))
                .splineToSplineHeading(dropPose, Math.toRadians(225))
                .addTemporalMarker(() -> {
                    lift.openClaw();
                    sleep(300);
                    lift.armAngle(0);
                })
                .waitSeconds(0.2)
                .build();
        TrajectorySequence left = robot.trajectorySequenceBuilder(cycle.end())
                .addTemporalMarker(() -> {
                    robot.vee.setPosition(0.6);
                    lift.setHeight(0);
                })
                .splineToLinearHeading(new Pose2d(60, 12, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence middle = robot.trajectorySequenceBuilder(cycle.end())
                .addTemporalMarker(() -> {
                    robot.vee.setPosition(0.6);
                    lift.setHeight(0);
                })
                .splineToLinearHeading(new Pose2d(36, 24, Math.toRadians(0)), Math.toRadians(90))
                .build();
        TrajectorySequence right = robot.trajectorySequenceBuilder(cycle.end())
                .addTemporalMarker(() -> {
                    robot.vee.setPosition(0.6);
                    lift.setHeight(0);
                })
                .splineToLinearHeading(new Pose2d(12, 12, Math.toRadians(0)), Math.toRadians(180))
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        lift.init(hardwareMap);
        lift.claw.setPosition(0.95);
        // robot.vee.setPosition(0.6);
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
                        telemetry.addData("Tag #", tagOfInterest.id);
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
        if(tagOfInterest != null){
            position = tagOfInterest.id;
        }
        robot.vee.setPosition(1);
        robot.followTrajectorySequence(preload);
        while (stack_height >= 1){
            lift.setHeight(heights[stack_height]);
            robot.followTrajectorySequence(cycle);
            stack_height--;
        }
        if (position == 1){
            robot.followTrajectorySequence(left);
        }
        else if (position == 3){
            robot.followTrajectorySequence(right);
        }
        else {
            robot.followTrajectorySequence(middle);
        }

    }

}
