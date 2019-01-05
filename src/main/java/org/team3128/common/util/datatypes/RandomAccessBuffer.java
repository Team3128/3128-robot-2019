package org.team3128.common.util.datatypes;


/**
 * 
 * Circular queue with a fixed size.
 * Supports random access.
 * The first element enqueued has index zero, and enqueueing another element will increment existing elements' indexes by 1.
 * enqueue()ing more than size elements will overwrite the highest-index element.
 * 
 * This class copyright (c) Jamie Smith
 */
public class RandomAccessBuffer<T>
{
	private Object[] elements;

    /**
     * current "zero" index
     */
    private int currentStartIndex;
    
    
    private int size = 0;

    /**
     * Get the actual size of the buffer (how many elements have been enqueued).
     * @return
     */
    public int getSize()
    {
        return size;   
    }

    /**
     * Get the last valid index in the buffer.  Same as getMaxSize() - 1. 
     * @return The highest index that can be used in the buffer.  -1 if the max size is 0.
     */
    public int getLastIndex()
    {
    	return size == 0 ? 0 : size-1;
    }

    /**
     * Get the maximum number of elements that can be inserted before old elements get overwritten.
     * @return
     */
    public int getMaxSize()
    {
    	return elements.length;
    }

	public RandomAccessBuffer(int size)
    {
        elements = new Object[size];

        currentStartIndex = size == 0 ? 0 : size - 1;
    }

	/**
	 * Add an element at the start.  
	 * 
	 * If the buffer is full, this shifts it backward. 
	 * 
	 * In other words, the element at the end will be removed, all other elements will have their indexes incremented by 1, and this element will be added at index 0.
	 * @param element
	 */
    public void enqueue(T element)
    {
        if (elements.length == 0)
        {
            throw new IllegalArgumentException("Tried to add element \"" + element.toString() + "\" to 0 length buffer!");
        }

        if(size < elements.length)
        {
            ++size;
        }

        //decrement start index
        if(currentStartIndex == 0)
        {
            currentStartIndex = elements.length - 1;
        }
        else
        {
            --currentStartIndex;
        }
        
        elements[currentStartIndex] = element;

    }

    /** 
    * Calculate the index in the internal array of the element which appears to be at apparentIndex.
    * Does validation, and throws an exception if index is invalid
    */
    private int calcInternalIndex(int apparentIndex)
    {
        if (apparentIndex >= size)
        {
            throw new ArrayIndexOutOfBoundsException("Index: " + apparentIndex + ", size: " + size);
        }

        int internalIndex = apparentIndex + currentStartIndex;
        if (internalIndex >= elements.length)
        {
        	internalIndex -= elements.length - 1;
        }

        return internalIndex;
    }

    /**
     * Get the element at index.
     * @param index
     * @return
     */
    @SuppressWarnings("unchecked")
	public T get(int index)
    {
    	return (T) elements[calcInternalIndex(index)];
    }
    
    /**
     * Set the element at index to value.
     * 
     * Does not shift the buffer, use enqueue() for that.
     * 
     * If  index >= size, then size will be increased to index + 1, and all of the elements in between will be filled with nulls.
     * @param index
     * @param value
     */
    public void set(int index, T value)
    {   
        if(index >= size && index < getMaxSize())
        {
        	size = index + 1; 
        }
        
        elements[calcInternalIndex(index)] = value;
        
    }

    //Changes the fixed size of the queue
    //Extra space is added at, and surplus elements are removed from, the end.
    public void resize(int newSize)
    {
        Object[] newBackingArray = new Object[newSize];

        //we'll use apparent indexes for this, it's too hard to use real indexes
        for(int index = 0; index < Math.min(size, newSize); ++index)
        {
        	//note: on the new array, since the start index is 0, apparent indexes and real indexes are the same
            newBackingArray[index] = get(index);
        }

        //now use the new array
        currentStartIndex = 0;
        elements = newBackingArray;
        size = newSize;
        
    }

    @Override
    public String toString()
    {
        StringBuilder retval = new StringBuilder();
        retval.append('{');

        //we'll use apparent indexes for this, it's too hard to use real indexes
        for (int index = 0; index < size; ++index)
        {
            retval.append('[');
            retval.append(String.valueOf(get(index)));
            retval.append(']');

            if(index < size - 1)
            {
                retval.append(", ");
            }
        }

        retval.append('}');
//        
//        retval.append(" iind: ");
//        retval.append(currentStartIndex);
//        retval.append(Arrays.toString(elements));
        
        
        return retval.toString();
    }
}
