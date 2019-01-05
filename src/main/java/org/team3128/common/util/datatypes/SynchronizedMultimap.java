package org.team3128.common.util.datatypes;

import java.util.HashSet;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Our homegrown replacement for a Guava multimap.
 * 
 * Currently, this is needed to make the ControlWatcher work.
 * @author Jamie
 *
 * @param <Key>
 * @param <Value>
 */
public class SynchronizedMultimap<Key, Value>
{
	ConcurrentHashMap<Key, HashSet<Value>> _map;
	
	public SynchronizedMultimap()
	{
		_map = new ConcurrentHashMap<Key, HashSet<Value>>();
	}
	
	/**
	 * insert a(nother) value for a key
	 * @param key
	 * @param value
	 */
	public void put(Key key, Value value)
	{
		HashSet<Value> prevValues = _map.get(key);
		if(prevValues == null)
		{
			prevValues = new HashSet<Value>();
		}
		
		prevValues.add(value);
		_map.put(key, prevValues);
	}
	
	/**
	 * remove all keys and values from the multimap
	 */
	public void clear()
	{
		_map.clear();
	}
	
	/**
	 * remove the key and all of its values from the map
	 */
	public void removeAll(Key key)
	{
		_map.remove(key);
	}
	
	/**
	 * get a list of values for a key
	 * @param key
	 * @return
	 */
	public HashSet<Value> get(Key key)
	{
		return _map.get(key);
	}
	
	
}	
