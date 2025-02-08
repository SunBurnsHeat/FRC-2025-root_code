package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.WinchConfigs;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WinchConstants;

public class WinchSubsystem extends SubsystemBase{
    private final SparkMax winchMax = new SparkMax(WinchConstants.kWinchCANID, MotorType.kBrushless);

    private final RelativeEncoder winchEncoder = winchMax.getEncoder();
    
    private final XboxController controller = new XboxController(OIConstants.kCoPilotControllerPort);

    public WinchSubsystem(){
        CommandScheduler.getInstance().registerSubsystem(this);

        winchMax.configure(WinchConfigs.winchMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        winchEncoder.setPosition(0);
    }

    public void runWinch(double setPoint){
        winchMax.set(setPoint);
    }

    @Override
    public void periodic() {
        if(Math.abs(controller.getLeftY()) < 0.015) {
            runWinch(WinchConstants.kIdleSpeed);
        }
        else {
            runWinch(controller.getLeftY()*WinchConstants.kWinchSpeed);
        }
    }

}
