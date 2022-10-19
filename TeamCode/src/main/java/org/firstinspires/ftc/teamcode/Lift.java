package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Lift {
    private DcMotorEx lift_left;
    private DcMotorEx lift_right;
    private DcMotorEx arm;


    public void init(HardwareMap hardwareMap){

        lift_left = hardwareMap.get(DcMotorEx.class, "left_lift");
        lift_right = hardwareMap.get(DcMotorEx.class, "right_lift");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        lift_left.setTargetPosition(0);
        lift_right.setTargetPosition(0);
        arm.setTargetPosition(0);
        lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift_left.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_right.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeight(double height){
        double value = height / 112 * 537.7;
        int target = (int)value;
        lift_left.setTargetPosition(target);
        lift_right.setTargetPosition(target);
        lift_left.setVelocity(3898 * 0.5);
        lift_right.setVelocity(3898 * 0.5);
    }
    public void armAngle(double angle){
        double value = angle * (537.7 / 360);
        int target = (int)value;
        arm.setTargetPosition(target);
        arm.setVelocity(265);
    }

}
