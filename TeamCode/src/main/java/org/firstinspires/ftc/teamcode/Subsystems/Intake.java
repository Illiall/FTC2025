package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants;

public class Intake {
    //Sets the motor name
    DcMotor intake;
    public Intake(HardwareMap hardwareMap){
        //Sets the motor name in the controller
        intake = hardwareMap.get(DcMotor.class,"intake");
        //Reverses the motor if needed
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //Function for intaking game pieces
    public void intake(){
        intake.setPower(constants.intake_speed);
    }
    //Function for holding game pieces while idle
    public void hold(){
        intake.setPower(constants.holding_speed);
    }

    //Emergency Stop for intake
    public void stop(){
        intake.setPower(0);
    }
}
