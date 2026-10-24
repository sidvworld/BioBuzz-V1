
package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import java.util.EnumMap;
import java.util.Objects;
import java.util.function.Supplier;

public class StateMachine {
    public enum State {
        IDLE,
        INTAKE,
        SHOOT_POLLEN,
        INTAKE_AND_SHOOT
    }

    private final EnumMap<State, Supplier<Command>> enters = new EnumMap<>(State.class);
    private final EnumMap<State, Supplier<Command>> actives = new EnumMap<>(State.class);
    private final EnumMap<State, Supplier<Command>> exits = new EnumMap<>(State.class);
    private final EnumMap<State, EnumMap<State, Supplier<Command>>> transitions = new EnumMap<>(State.class);
    private State state, requested, destination;
    private Command running;

    public StateMachine(){
        for (State s : State.values()){
            configureState(s, StateMachine::none, StateMachine::none, StateMachine::none);
            transitions.put(s, new EnumMap<>(State.class));
        }
    }

    public static Command none(){
        return instant(() -> {});
    }

    public void configureState(State state, Supplier<Command> enter, Supplier<Command> active, Supplier<Command> exit){
        enters.put(state, enter);
        actives.put(state, active);
        exits.put(state, exit);
    }

    public void addTransition(State from, State to, Supplier<Command> transition){
        Objects.requireNonNull(transitions.get(from)).put(to, transition);
    }

    public void initialize(State initial) {
        stop();
        requestState(initial);
        update();
    }

    public void requestState(State next){
        requested = next;
    }

    public Command requestStateCmd(State next){
        return instant(() -> requestState(next));
    }

    public State getState() { return state; }
    public State getRequestedState() { return requested; }

    public void update() {
        if (requested == null) return;

        if (requested == destination) {
            return;
        }

        if (running != null) {
            running.cancel();
        }

        final State next = requested;

        Supplier<Command> bridge = state == null
                ? StateMachine::none
                : Objects.requireNonNull(transitions.get(state))
                .getOrDefault(next, StateMachine::none);

        assert bridge != null;
        running = sequential(
                state == null
                        ? none()
                        : Objects.requireNonNull(exits.get(state)).get(),
                bridge.get(),
                instant(() -> state = next),
                Objects.requireNonNull(enters.get(next)).get(),
                Objects.requireNonNull(actives.get(next)).get(),
                infinite(() -> {})
        ).setPriority(1);

        destination = next;
        running.schedule();
    }

    public void stop() {
        if (running != null) running.cancel();
        running = null;
        state = requested = destination = null;
    }
}
