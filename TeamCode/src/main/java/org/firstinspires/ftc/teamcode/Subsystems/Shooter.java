package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants;

public class Shooter {
    //Names the motor
    DcMotor shooter;

    public Shooter(HardwareMap hardwareMap){
        //sets the name in the controller
        shooter = hardwareMap.get(DcMotor.class,"shooter");

        //Reverses the motor if needed
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //Function for shooting the game pieces
    public void shoot(){
        shooter.setPower(constants.shooting_speed);
    }
    //Function for stoping the shooter|Emergency Stop for shooting
    public void stop(){
        shooter.setPower(0);
    }
}
