// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.gyro;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.gyro.Gyro.GyroIOInputs;

/** IO implementation for Pigeon2 */
public class GyroHardware implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroHardware() {

        pigeon = new Pigeon2(Constants.MotorIdConstants.GYRO_CAN_ID, "rio");
        Optional<Alliance> ally = DriverStation.getAlliance();
    
        if (ally.get() == Alliance.Red) {
            this.setYaw(Math.toRadians(180.0)); 
        } else if (ally.get() == Alliance.Blue) {
            this.setYaw(0.0); 
        } else {
            this.setYaw(0.0); 
        }
        

    }
    
        
    

    public double getYaw() {
        return Units.degreesToRadians(pigeon.getYaw().getValueAsDouble());
    }

    public void setYaw(double yaw) {
        pigeon.setYaw(Units.radiansToDegrees(yaw));
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPositionRad = pigeon.getYaw().getValueAsDouble();
    }

    public StatusSignal<edu.wpi.first.units.measure.AngularVelocity> getAngularVelocity() {
        return pigeon.getAngularVelocityZDevice();
    }
}