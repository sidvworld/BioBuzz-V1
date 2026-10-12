package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;

import java.util.EnumMap;
import java.util.function.Supplier;

public class StateMachine {
    public enum State {
        OFF,
        IDLE,
        INTAKE,
        SHOOT
    }

    private final EnumMap<State, Supplier<Command>> enters = new EnumMap<>(State.class);
    private final EnumMap<State, Supplier<Command>> actives = new EnumMap<>(State.class);
    private final EnumMap<State, Supplier<Command>> exits = new EnumMap<>(State.class);
    private final EnumMap<State, EnumMap<State, Supplier<Command>>> transitions = new EnumMap<>(State.class);
    private State state, requested, destination;


}
