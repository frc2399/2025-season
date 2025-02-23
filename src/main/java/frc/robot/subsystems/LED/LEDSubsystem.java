package frc.robot.subsystems.LED;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;

public class LEDSubsystem extends SubsystemBase {
    private static final int kPort = 9;
    private static final int kLength = 15;
    private static final Distance kLEDSpacing = Meters.of(1 / kLength);

    private static AddressableLED led;
    private static AddressableLEDBuffer ledBuffer;
    private static boolean isAutonomous;
    private int rainbowFirstPixelHue;
    private CoralIntakeSubsystem coralIntake;
    private AlgaeIntakeSubsystem algaeIntake;
    private LEDPattern pink;
    private LEDPattern blue;

    public LEDSubsystem(CoralIntakeSubsystem coralIntake, AlgaeIntakeSubsystem algaeIntake) {
        this.coralIntake = coralIntake;
        this.algaeIntake = algaeIntake;
        led = new AddressableLED(kPort);
        ledBuffer = new AddressableLEDBuffer(kLength);
        isAutonomous = true;
        rainbowFirstPixelHue = 0;
        pink = LEDPattern.solid(Color.kHotPink);
        blue = LEDPattern.solid(Color.kBlue);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    private void rainbow() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
            System.out.println("colorful");
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;

        led.setData(ledBuffer);
        led.start();
    }

    public void turnTeleop() {
        isAutonomous = false;
    }

    public void teleopLed() {
        if (coralIntake.setHasCoral(coralIntake.hasCoral) == true)
        {
            pink.applyTo(ledBuffer);
        } else if (algaeIntake.setHasAlgae(algaeIntake.hasAlgae) == true) {
            blue.applyTo(ledBuffer);
        }
        led.setData(ledBuffer);
        led.start();
    }
      
    public void periodic() {
        if (isAutonomous) {
            rainbow();
        }
        else {
            teleopLed();
        }
    } 
}  