package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RightMid extends LinearOpMode {
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
    int[] heights = {0, 35, 70, 115, 140};

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
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        Pose2d startPose = new Pose2d(-31.1, 61.6, Math.toRadians(90));
        Pose2d dropPose = new Pose2d(-27.7, 18.8, Math.toRadians(215));
        Vector2d stackPose = new Vector2d(-62, 10.5);
        int position = 2;
        robot.setPoseEstimate(startPose);
        telemetry.addData( "Passed", "1");
        telemetry.update();

        TrajectorySequence preload = robot.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.backArmSensor();
                    lift.setHeight(390);
                })
                .lineToConstantHeading(new Vector2d(-36, 30))
                //.splineToConstantHeading(new Vector2d(36, 25), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-27.4,4.7, Math.toRadians(155)), Math.toRadians(335),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.6))
                .addTemporalMarker(() -> {
                    lift.openClaw();
                    sleep(200);
                    lift.armAngle(0);
                })
                .waitSeconds(0.2)
                .build();
        TrajectorySequence cycle = robot.trajectorySequenceBuilder(preload.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    lift.openClaw();
                })
                .splineToSplineHeading(new Pose2d(-54, 9.5, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(stackPose, Math.toRadians(180))
                .addTemporalMarker(() -> {
                    lift.closeClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    lift.setHeight(390);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    lift.backArmSensor();
                    lift.setHeight(160);
                })
                .lineToConstantHeading(new Vector2d(-54, 9.5))
                .splineToSplineHeading(new Pose2d(-27.7, 20.8, Math.toRadians(215)), Math.toRadians(25))
                .build();
        TrajectorySequence cycle2 = robot.trajectorySequenceBuilder(cycle.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    lift.openClaw();
                })
                .splineToSplineHeading(new Pose2d(-54, 9.5, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(stackPose, Math.toRadians(180))
                .addTemporalMarker(() -> {
                    lift.closeClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    lift.setHeight(390);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    lift.backArmSensor();
                    lift.setHeight(160);
                })
                .lineToConstantHeading(new Vector2d(-54, 9.5))
                .splineToSplineHeading(new Pose2d(-27.7, 20.8, Math.toRadians(215)), Math.toRadians(25))
                .build();
        TrajectorySequence cycle3 = robot.trajectorySequenceBuilder(cycle2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    lift.openClaw();
                })
                .splineToSplineHeading(new Pose2d(-54, 9.5, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(stackPose, Math.toRadians(180))
                .addTemporalMarker(() -> {
                    lift.closeClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    lift.setHeight(390);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    lift.backArmSensor();
                    lift.setHeight(160);
                })
                .lineToConstantHeading(new Vector2d(-54, 9.5))
                .splineToSplineHeading(dropPose, Math.toRadians(25))
//                .addTemporalMarker(() -> {
//                    lift.openClaw();
//                    sleep(800);
//                    lift.armAngle(0);
//                })
//                .waitSeconds(0.2)
                .build();
        TrajectorySequence right = robot.trajectorySequenceBuilder(cycle2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.vee.setPosition(0.4);
                    lift.setHeight(0);
                })
                .splineToSplineHeading(new Pose2d(-60, 10.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL*2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*2))
                .build();
        TrajectorySequence middle = robot.trajectorySequenceBuilder(cycle2.end())
                .addTemporalMarker(() -> {
                    robot.vee.setPosition(0.4);
                    lift.setHeight(0);
                })
                .forward(8.5)
                .build();
        TrajectorySequence left = robot.trajectorySequenceBuilder(cycle2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    lift.openClaw();
                })
                .splineToSplineHeading(new Pose2d(-54, 9.5, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(stackPose, Math.toRadians(180))
                .addTemporalMarker(() -> {
                    lift.closeClaw();
                })
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-30, 9.5))
                .addTemporalMarker(() -> {
                    lift.setHeight(390);
                    lift.backArmSensor();
                })
                .splineToSplineHeading(new Pose2d(-3.7, 19.8, Math.toRadians(215)), Math.toRadians(35))
                .addTemporalMarker(() -> {
                    lift.openClaw();
                    sleep(300);
                    lift.armAngle(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.setHeight(0);
                    robot.vee.setPosition(0.4);
                })
                .forward(8)
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        lift.init(hardwareMap);
        lift.closeClaw();
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
        lift.setHeight(heights[stack_height]);
        robot.followTrajectorySequence(cycle);
        lift.openClaw();
        sleep(200);
        lift.armAngle(0);
        sleep(100);
        stack_height--;
        lift.setHeight(heights[stack_height]);
        robot.followTrajectorySequence(cycle2);
        lift.openClaw();
        sleep(200);
        lift.armAngle(0);
        sleep(100);
        stack_height--;
        while (stack_height >= 1){
            lift.setHeight(heights[stack_height]);
            robot.followTrajectorySequence(cycle3);
            lift.openClaw();
            sleep(200);
            lift.armAngle(0);
            sleep(100);
            stack_height--;
        }
        lift.setHeight(0);
        if (position == 3){
            robot.followTrajectorySequence(cycle2);
            lift.openClaw();
            sleep(200);
            lift.armAngle(0);
            sleep(100);
            robot.followTrajectorySequence(right);
        }
        else if (position == 2){
            robot.followTrajectorySequence(cycle2);
            lift.openClaw();
            sleep(200);
            lift.armAngle(0);
            sleep(100);
            robot.followTrajectorySequence(middle);
        }
        else {
            robot.followTrajectorySequence(left);
        }

    }

}
