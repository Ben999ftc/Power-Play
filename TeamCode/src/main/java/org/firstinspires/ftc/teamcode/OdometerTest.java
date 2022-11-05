package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@TeleOp
public class OdometerTest extends LinearOpMode {
    private TrajectoryFollower follower;
    private TrajectorySequenceRunner trajectorySequenceRunner;
    private StandardTrackingWheelLocalizer standardTrackingWheelLocalizer;
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);


    @Override
    public void runOpMode() {
        standardTrackingWheelLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);


        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("left:", standardTrackingWheelLocalizer.getLeftOdometer());
            telemetry.addData("right:", standardTrackingWheelLocalizer.getRightOdometer());
            telemetry.addData("back:", standardTrackingWheelLocalizer.getBackOdometer());
            telemetry.update();
        }
    }
}