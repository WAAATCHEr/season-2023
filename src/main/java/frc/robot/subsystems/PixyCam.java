package frc.robot.subsystems;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pixy2api.Pixy2;
import pixy2api.Pixy2CCC;
import pixy2api.Pixy2Video;
import pixy2api.Pixy2CCC.Block;
import pixy2api.links.SPILink;


/**
 * COMPETITION READY
 * 
 * All vision code for PixyCam2.
 */
public class PixyCam extends SubsystemBase {

    private static PixyCam instance;

    public static PixyCam getInstance() {
        if (instance == null) {
            instance = new PixyCam();
        }
        return instance;
    }

    // FIELDS
    Pixy2 pixy;
    Pixy2CCC pixyCCC;
    Pixy2Video pixyVideo;

    /**
     * Used for goal speeds to smooth.
     */
    double[] avg = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    private Block biggestObject = null;

    private PixyCam() {
        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();
        pixyCCC = pixy.getCCC();
        pixyVideo = pixy.getVideo();

        // 0- off, 1- on
        // First param is headlamps (white), second is RGB LED
        // pixy.setLamp((byte) 0, (byte) 1);
    }
  
    public Block getBiggestObject() {
        return biggestObject;
    }

    public Block setBiggestObject(Block b) {
        biggestObject = b;
        return biggestObject;
    }

    /**
     * Gets blocks of type [Constants.Signature]
     */
    public List<Block> getBlocksOfType(int signature) {
        List<Block> output = new ArrayList<>();
        pixyCCC.getBlocks(false, signature, 25);

        for (Block b : pixyCCC.getBlocks()) {
            output.add(b);
        }

        return output;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Block getLargestBlock(List<Block> blocks) {
        Block biggest = null;
        double largestArea = 0;
        for (Block b : blocks) {
            double area = (b.getWidth() * b.getHeight());
            if (area >= largestArea) {
                biggest = b;
                largestArea = area;
            }
        }
        if (biggest == null) {
            return blocks.get(0);
        }
        
        return biggest;
    }

    public Command printCommand(){
        return new RepeatCommand( new RunCommand(() -> System.out.println(this.getBlocksOfType(2).toString()), this));
    }

    

}