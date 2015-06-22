/**
 * 
 */
package mil.navy.nrl.ncarai.iss.mds.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import javax.swing.JFileChooser;

/**
 * @author magda
 *
 */
public class BatteryBankLogParser {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		File f = new File("");
		final JFileChooser fc = new JFileChooser();
		boolean done = false;
		
		while(!done && f.exists() == false)
		{
			int returnVal = fc.showOpenDialog(null);

        	if (returnVal == JFileChooser.APPROVE_OPTION) {
            		f = fc.getSelectedFile();
        	} 
        	else if (returnVal == JFileChooser.CANCEL_OPTION) {
        		done = true;
        	}
		}
		
		try {
			if (!done) {
				Scanner infile = new Scanner(f);
				double duration = 0.0;
				
				// Pattern: date duration
				while(infile.hasNext()) {
					String[] data = infile.nextLine().split(" ");
										
					duration += Double.valueOf(data[1]);

				}
				
				// Print out the summary
				System.out.println(String.format("The banks has been on for a total of %.4f", duration));
			}
		}
		catch (FileNotFoundException e) {
			System.err.println("File not found.");
		}
	}

}
