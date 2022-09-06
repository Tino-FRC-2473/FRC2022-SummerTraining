package frc.robot;

import java.util.HashMap;

public class FSMCoordinator {
	HashMap<Class, Object> FSMs = new HashMap<>();

	public final void addSystems(Object... systems){
		for(Object system : systems){
			if(!FSMs.containsKey(systems.getClass())){
				FSMs.put(systems.getClass(), system);
			}
		}
	}

	public final Object getSystem(Class key){
		return FSMs.get(key);
	}
}
