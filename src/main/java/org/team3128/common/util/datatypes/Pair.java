package org.team3128.common.util.datatypes;

/**
 * This class is useful when one wants to return two values from one function. 
 * I can't understand why it is not in the Java Standard Library .
 * 
 * 
 * Supports equals() and hashCode() comparison, so can be used in HashMap's.
 * @author Jamie
 *
 */
public class Pair<L, R>
{
	public L left;
	
	public R right;
	
	public Pair(L left, R right)
	{
		this.left = left;
		this.right = right;
	}

	public Pair()
	{

	}
	
	@Override
	public boolean equals(Object obj)
	{
		if(obj instanceof Pair<?, ?>)
		{
			Pair<?, ?> other = ((Pair<?, ?>)obj);
			return other.left.equals(left) && other.right.equals(right);
		}
		
		return false;
	}
	
	@Override
	public int hashCode()
	{
		return left.hashCode() * 41 + right.hashCode();
	}
}
