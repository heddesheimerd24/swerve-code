package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics {
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

    //pcmCompressor.enableDigital();
    //pcmCompressor.disable();

    //boolean enabled = pcmCompressor.enabled();
    boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    //double current = pcmCompressor.getCompressorCurrent();


    Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    Solenoid exampleSolenoidPH = new Solenoid(PneumaticsModuleType.REVPH, 1);

    //exampleSolenoidPCM.set(true);
    //exampleSolenoidPCM.set(false);
}
