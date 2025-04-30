// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }
}
