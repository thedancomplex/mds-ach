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
public class MotionLogParser {

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
				HashMap<Integer, Double> durations = new HashMap<Integer, Double>();
				
				// Pattern: time level name : node duration
				while(infile.hasNext()) {
					String[] line = infile.nextLine().split(":");
					
					String[] data = line[1].trim().split(" ");
					
					Integer node = Integer.valueOf(data[0].substring(2), 16);
					Double time = Double.valueOf(data[1]);

					if (durations.containsKey(node)) {
						Double total = durations.get(node);
						total += time;
						durations.put(node, total);
					}
					else {
						durations.put(node, time);
					}
				}
				
				// Print out the summary
				for (Entry<Integer, Double> entry : durations.entrySet()) {
					
					System.out.println(String.format("0x%04x", entry.getKey()) + "\t" + 
							String.format("%.4f", entry.getValue()));
				}
			}
		}
		catch (FileNotFoundException e) {
			System.err.println("File not found.");
		}
	}

}
