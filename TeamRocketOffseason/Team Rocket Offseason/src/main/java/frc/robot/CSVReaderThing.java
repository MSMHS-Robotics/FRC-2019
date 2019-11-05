package frc.robot;

import java.io.FileReader;
import com.opencsv.CSVReader;

public class CSVReaderThing {
    public static int readRow(String file) { 
    try { 
  
        // Create an object of filereader 
        // class with CSV file as a parameter. 
        FileReader filereader = new FileReader(file); 
  
        // create csvReader object passing 
        // file reader as a parameter 
        CSVReader csvReader = new CSVReader(filereader); 
        String[] nextRecord; 
  
        // we are going to read data line by line 
        while ((nextRecord = csvReader.readNext()) != null) { 
            for (String cell : nextRecord) { 
                nextRecord = csvReader.readNext(); 
            } 
        } 
    } 
    catch (Exception e) { 
        e.printStackTrace(); 
    } 
}
}