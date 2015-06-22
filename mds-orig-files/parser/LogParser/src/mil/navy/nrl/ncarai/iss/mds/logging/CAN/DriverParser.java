package mil.navy.nrl.ncarai.iss.mds.logging.CAN;
import java.io.FileNotFoundException;
import java.lang.ClassNotFoundException; 
import javax.swing.JFrame;

public class DriverParser 
{
	public static void main(String[] args) throws FileNotFoundException, ClassNotFoundException
	{
	    JFrame frame = new JFrame("Parsed Log... Incoming Messages on Left, Outgoing Messages on Right");
        frame.setSize(2000, 1500);
        frame.setContentPane(new PanelParser());
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setVisible(true);
	}
}