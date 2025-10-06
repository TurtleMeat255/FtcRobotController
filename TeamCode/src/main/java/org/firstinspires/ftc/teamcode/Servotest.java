package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Servotest extends LinearOpMode {
    private CRServo servo;
    private CRServo servo2;


    @Override
    public void runOpMode() {
        servo = hardwareMap.get(CRServo.class, "servo");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
    }
}