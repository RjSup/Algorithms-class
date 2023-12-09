package comp50172023v2;

/**
 *
 * @author D Lightfoot 2023
 */
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Scanner;

public class Network {
    public static final double NO_LINK = Double.MAX_VALUE; // "infinity"
    public  final int MAX_STATIONS;
    private   int numStations; // 0 <= numStations <= MAX_STATIONS
    private  final IStationInfo [] stations ;
    private  final double [][] distance;
    
    /**
     * @param x0, y0, x1, y1
     * @pre true
     * @return straight-line distance between (x0, y0) and (x1, y1)
     */
    public  double pythogoras(double x0, double y0, double x1, double y1){
     return Math.sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
  }
  
 public Network (int capacity ){
    MAX_STATIONS = capacity;
    stations = new StationInfo[MAX_STATIONS];
    numStations = 0;
    distance = new double[MAX_STATIONS][MAX_STATIONS];
   // make all distances initially NO_LINK
        for (int i = 0; i != MAX_STATIONS; i++) {
            for (int j = 0; j != MAX_STATIONS; j++) {
                distance[i][j] = NO_LINK;
            }
        }
 }
 
 /**
     * @pre true
     * @return number of stations
     */public int getNumStations(){return numStations;}
 
/**
     * @param i the index in the station array
     * @pre i must be in range 0..getNumStations()-1
     * @return stationInfo at index i
     */
 public IStationInfo getStationInfo(int i) {
    assert 0 <= i && i < numStations;
   return stations[i];
 }
   /**
     * @param name the station name to locate
     * @pre name is not null and not empty string
     * @return index of name in list of station or numStations if not found
     */
     public int indexOfStation(String name) {
       assert name != null & !name.trim().equals(""): " name must not be null or empty string";
        int i = 0;
        while (i != numStations && !name.equals(stations[i].getName())) i++;
        return i;
    }
     
 void readFileStream(FileInputStream fStream) throws IOException {
        Scanner scan = new Scanner(fStream);
        while (scan.hasNext()) {
            String word = scan.next();

            if (word.equals("station")) {
                if (numStations == MAX_STATIONS) {
                    throw new IOException("too many stations");
                }
                String name = scan.next();
                int x = scan.nextInt();
                int y = scan.nextInt();
                if (!(0 <= x && x < 256 && 0 <= y && y < 256)) {
                    throw new IOException(" x and/or y not in range");
                }
                if (indexOfStation(name) != numStations) {
                    throw new IOException("station name " + name + " already defined");
                }
                StationInfo s = new StationInfo(name, x, y);
                stations[numStations] = s;
                numStations++;

            } else if (word.equals("link")) {
                String fromName = scan.next();
                int fromIndex = indexOfStation(fromName);
                if (fromIndex == numStations) {
                    throw new IOException("station name " + fromName + " not defined");
                }
                String toName = scan.next();
                int toIndex = indexOfStation(toName);
                if (toIndex == numStations) {
                    throw new IOException("station name " + toName + " not defined");
                }
                if (fromIndex == toIndex) {
                    throw new IOException("loop for station " + toName);
                }
                double d = scan.nextDouble();
                if (d <= 0.0) {
                    throw new IOException("distance must not be negative");
                }
                distance[fromIndex][toIndex] = d;
                distance[toIndex][fromIndex] = d;
                System.out.println(fromName + " -> "+ toName + ": " + d);

            } else {
                throw new IOException("Syntax Error in File");
            }
        }
    }
    
  /**
     * @param filename -- name of input text file
     * @pre filename is a valid well formed file of network information
     * @post network has been loaded from file
     */
   public void readNetwork(String filename){
        try {
            System.out.println("Reading file " + filename);
            readFileStream(new FileInputStream(filename));
        } catch (Exception e) {
            System.out.println("reading of file " + filename + " failed");
            System.out.print(e);
        }
  }
  
 /**
     * @post shows information held in network
     */
   public void display( ) {
     System.out.println("stations");
        for (int i = 0; i != numStations; i++) {
            System.out.println(i + ": " + stations[i].toString());
        }
        System.out.println("links");
        
    for (int i = 0; i != numStations; i++) {
            for (int j = 0; j != numStations; j++) {
                System.out.print(" " + i + " " + j + " ");
                if (distance[i][j] == NO_LINK) {
                    System.out.print("--  ");
                } else {
                    System.out.print(distance[i][j]);
                }
            }
            System.out.println();
        }
 }
 
   /**
     * @param fromIndex, toIndex indexes in the station array
     * @pre fromIndex, toIndex must be in range 0..getNumStations()-1
     * @return recorded distance between station at fromIndex and station at toIndex
     */
     public double getDistance(int fromIndex, int toIndex) {
      assert 0 <= fromIndex && fromIndex < numStations
              && 0 <= toIndex && toIndex < numStations;
      return distance[fromIndex][toIndex];
   }
}