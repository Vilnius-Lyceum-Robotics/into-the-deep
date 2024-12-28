package org.firstinspires.ftc.teamcode.helpers.controls;

@FunctionalInterface
public interface QuadConsumer<T1, T2, T3, T4> {
    void accept(T1 t1, T2 t2, T3 t3, T4 t4);
}