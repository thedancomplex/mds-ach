/**
 * 
 */
package mil.navy.nrl.ncarai.iss.mds.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.Scanner;
import java.util.Map.Entry;

import javax.swing.JFileChooser;

/**
 * @author magda
 *
 */
public class BatteryServerLogParser {

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
				
				// Pattern: day month day time zone year duration([hh:]mm:ss) process <flags> <parameters>
				while(infile.hasNext()) {
					String line = infile.nextLine();
					
					if (!line.isEmpty()) {
						String[] stuff = line.split(" ");
						
						if (stuff.length > 6) {
							String[] stamp = stuff[6].trim().split(":");
													
							short index = 0;
							if (stamp.length == 3) {
								duration += Double.valueOf(stamp[0]);
								index++;
							}
							duration +=  
										(Double.valueOf(stamp[index++]) / 60.0) +
										(Double.valueOf(stamp[index++]) / (3600));
							}

					}
				}
				
				// Print out the summary					
				System.out.println(String.format("The battery server has been running for a total of %.4f hours.", duration));
			}
		}
		catch (FileNotFoundException e) {
			System.err.println("File not found.");
		}
	}

}
