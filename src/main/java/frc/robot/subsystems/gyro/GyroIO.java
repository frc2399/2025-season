// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.StatusSignal;

public interface GyroIO {

    public double getYaw();

    public void setYaw(double yaw);

    public StatusSignal<edu.wpi.first.units.measure.AngularVelocity> getAngularVelocity();

}