package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Lift {
    private DcMotor lift_left;
    private DcMotor lift_right;
    private DcMotor arm;

    double currentheight = 0;
    double currentangle = 0;


    public void init(HardwareMap hardwareMap){

        lift_left = hardwareMap.get(DcMotor.class, "left_lift");
        lift_right = hardwareMap.get(DcMotor.class, "right_lift");
        arm = hardwareMap.get(DcMotor.class, "arm");

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

    public void setHeight(double height, double power){
        double value = height / 112 * 537.7;
        int target = (int)value;
        lift_left.setTargetPosition(target);
        lift_right.setTargetPosition(target);
        if(target < currentheight) {
            lift_left.setPower(power * -1);
            lift_right.setPower(power * -1);
        }
        else{
            lift_left.setPower(power);
            lift_right.setPower(power);
        }
        currentheight = target;
    }
    public void armAngle(double angle, double power){
        double value = angle * (537.7 / 360);
        int target = (int)value;
        arm.setTargetPosition(target);
        if(target < currentangle) {
            arm.setPower(power * -1);
        }
        else{
            arm.setPower(power);
        }
        currentangle = target;
    }

}
