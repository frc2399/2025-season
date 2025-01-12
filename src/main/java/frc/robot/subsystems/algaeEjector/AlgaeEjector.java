package frc.robot.subsystems.algaeEjector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeEjector extends SubsystemBase{
    
    private AlgaeEjectorIO io;

    public AlgaeEjector(AlgaeEjectorIO io) {
        this.io=io;
    }

    public Command setSpeed(double speed) {
       return this.run( () -> Commands.run(()->io.setSpeed(speed)));
    }
}