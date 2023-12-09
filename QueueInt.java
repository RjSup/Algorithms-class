package comp50172023v2; // or whatever
/**
 *
 * @author David Lightfoot 2023
 */
public class QueueInt {
   private int queue[];
    private int size; // 0 <= size <= capacity
    private final int capacity;
    private int front;  // 0 <= from < capacity
    private int back; // 0 <= from < capacity

    public QueueInt(int capacity) {
        assert capacity > 0;
        this.capacity = capacity;
        this.size = 0;
        this.queue = new int[capacity];
        this.front = 0;
        this.back = 0;
    }
    public int getCapacity() {return capacity;}
    public int getSize() {return size;}

    /**
     * @param n node to be added
     * @pre getSize() != getCapacity()
     * @post n has been added to back of queue
     */
    public void addToBack(int n) {
        assert getSize() != getCapacity();
        queue[back] = n;
        back = (back + 1) % capacity;
        size++;
    }

    /**
     * @pre getSize() != 0
     * @post element at front of queue has been removed
     * @return value that has been removed
     */
    public int removefromFront() {
        assert getSize() != 0;
        int result = queue[front];
        front = (front + 1) % capacity;
        size--;
        return result;
    }

     /**
     * @return queue as a string
     */
    public String toString() {
        String result = "";
        int j = front;
        for (int i = 0; i != size; i++) {
            result += ("[" + j + ", " + queue[j]) + "]";
            j = (j + 1) % capacity;
        }
        return result;
    }
}

