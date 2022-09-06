package frc.robot;

import java.util.HashMap;

public class FSMCoordinator {
	private final HashMap<Class<?>, Object> finiteStateMachines = new HashMap<>();

	/**
	 * Add finite state machines to the coordinator pool.
	 * @param systems all the systems to be added to the coordinator
	 */
	public final void addSystems(Object... systems) {
		for (Object system : systems) {
			if (!finiteStateMachines.containsKey(systems.getClass())) {
				finiteStateMachines.put(systems.getClass(), system);
			}
		}
	}

	/**
	 * Gets the instantiated FSM object of the specified FSM
	 *
	 * Example code for getting the state of another FSM(code within a finiteStateMachinesystem)
	 *
	 * ...
	 * ((FiniteStateMachine) coordinator.getSystem(FiniteStateMachine.class))
	 * 			.getCurrentState();
	 * ...
	 *
	 * @param key The class of the desired FSM object
	 * @return The instantiated FSM object of the specified FSM
	 */
	public final Object getSystem(Class<?> key) {
		return finiteStateMachines.get(key);
	}
}
