package comp50172023v2; // or whatever
/**
 * @author D Lightfoot 2023
 */
public class SetInt  { 
    protected int set[];
    private  int size; // 0 <= size <= capacity
    private final int capacity;
    /**
     * @param capacity --  capacity of this set
     * @pre capacity >= 0
     * @post new set of current size zero has been created
     */
    public SetInt(int capacity) {
       assert capacity > 0;
        this.capacity = capacity;
        this.size = 0;
        this.set = new int[capacity];
    }
    public int getCapacity() {return capacity;}
    public int getSize() {return size;}
    
    /**
     * @param x -- value to be sought
     * @pre true
     * @return true iff x is in list
     */
    public boolean contains(int x) {
        int i = 0;
        while (i != getSize() && set[i] != x) i++;
        return i != getSize();
    }
    /**
     * @param n node to be added
     * @pre !contains(n) !! getSize() != getCapacity()
     * @post contains(n)
     */
    public void include(int n) {
        assert contains(n) || getSize() != getCapacity() : "full" + getSize();
        if (!contains(n)) {
            set[size++] = n;
        }
    }
    /**
     * @pre n not in set or size != 0
     * @post !contains(n)
     */
    public void exclude(int n) {
        assert !contains(n) || getSize() != 0;
        int i = 0;
        while (i != getSize() && set[i] != n)  i++;
        if (i != getSize()) { // contains(n)
            set[i] = set[size - 1]; // swap in last element of set
            size--;
        }
    }
   /**
     * @return set as a string
     */
    public String toString() {
        String result = "";
        for (int i = 0; i != size; i++) {
            result += ("[" + i + ", " + set[i] + "]");
        }
        return result;
    }
}