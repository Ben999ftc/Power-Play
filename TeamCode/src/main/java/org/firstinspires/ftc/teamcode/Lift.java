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
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift_left.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_right.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void setHeight(double height){
        double ticks_per_mm = (384.5 / 112);
        double value = height * ticks_per_mm;
        int target = (int)value;
        lift_left.setTargetPosition(target);
        lift_right.setTargetPosition(target);
        lift_left.setVelocity(2785);
        lift_right.setVelocity(2785);
    }
    public void armAngle(double angle){
        double ticks_per_degree = (2786.2 / 360);
        double  value = (angle * ticks_per_degree);
        int target = (int)value;
        arm.setTargetPosition(target);
        arm.setVelocity(2000);

    }
    public double getAngle(){
        return (arm.getCurrentPosition() / (2786.2 / 360));
    }

    public double getHeight(){return lift_left.getCurrentPosition() / (384.5/112);}

}
