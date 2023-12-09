package comp50172023v2; // or whatever

/**
 *
 * @author David Lightfoot 2023
 */
public class StackInt  {
    private int stack[];
    private int size; // 0 <= size <= capacity
    private  final int capacity;
    
    public StackInt(int capacity) {
       assert capacity > 0; 
        this.capacity = capacity;
        this.size = 0;
        stack = new int[capacity];
    }
    public int getCapacity() {return capacity;}
    public int getSize() {return size;}
    /**
     * @param n node to be added
     * @pre getSize() != getCapacity()
     * @post n has been pushed on to top of stack
     */
    public void push(int n) {
        assert getSize() != getCapacity();
        stack[size] = n;
        size++;
    }

    /**
     * @pre getSize() != 0
     * @post element on top of stack has been removed
     * @return value that has been removed
     */
    public int pop() {
        assert getSize() != 0;
        size--;
        return stack[size];
    }

    /**
     * @pre getSize() != 0
     * @return value on top of stack
     */
 
    public int peek() {
        assert getSize() != 0;
        return stack[size - 1];
    }
}