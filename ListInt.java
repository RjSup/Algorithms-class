package comp50172023v2; // or whatever

/**
 *
 * @author D Lightfoot 2023
 */
public class ListInt  {
    private  int list[];
    private int size; // 0 <= size <= capacity
    private final int capacity;
    /**
     * @param capacity -- maximum capacity of this list
     * @post new list of current size zero has been created
     */
    public ListInt(int capacity) {
        // implements a bounded list of int values
       assert capacity > 0;
         this.capacity = capacity;
        this.size = 0;
        list = new int[capacity];
    }
    public int getCapacity() {return capacity;}
    public int getSize() {return size;}
    
/**
     * @param i index
     * @pre 0 <= i && i < getSize()
     * @return value in list at index i
     */
    public int get(int i) {
        assert 0 <= i && i < getSize();
        return list[i];
    }
    /**
     * @param n node to be added
     * @pre getSize() != getCapacity()
     * @post n has been appended to list
     */
    public void append(int n) {
        assert getSize() != getCapacity();
        list[size++] = n;
    }

    /**
     * @param x -- value to be sought
     * @pre true
     * @return true iff x is in list
     */
    public boolean contains(int x) {
        int i = 0;
        while (i != getSize() && list[i] != x) i++;
        return i != getSize();
    }
    
    @Override
    public String toString() {
       /**
     * @return list as a string
     */
        String s = "";
        int i = 0;
        while (i != size) {s += list[i];  i++;}
        return s;
    }
}

