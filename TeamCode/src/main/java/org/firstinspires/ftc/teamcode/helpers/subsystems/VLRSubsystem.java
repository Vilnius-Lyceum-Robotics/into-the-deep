package org.firstinspires.ftc.teamcode.helpers.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public abstract class VLRSubsystem<T extends VLRSubsystem<T>> extends SubsystemBase {
    private static final Map<Class<?>, VLRSubsystem<?>> instances = new HashMap<>();

    protected VLRSubsystem() {
        // Protected constructor to prevent direct instantiation
    }

    @SuppressWarnings("unchecked")
    public static <E extends VLRSubsystem<E>> E getInstance(Class<E> clazz) {
        if (!instances.containsKey(clazz)) {
            try {
                E instance = clazz.getDeclaredConstructor().newInstance();
                instances.put(clazz, instance);
            } catch (Exception e) {
                throw new RuntimeException("Error creating instance of " + clazz.getName(), e);
            }
        }
        return (E) instances.get(clazz);
    }

    protected abstract void initialize(HardwareMap hardwareMap);

    public static void initializeAll(HardwareMap hardwareMap) {
        for (VLRSubsystem<?> subsystem : instances.values()) {
            subsystem.initialize(hardwareMap);
        }
    }

    @SuppressWarnings("unchecked")
    public static void requireSubsystems(Class<? extends VLRSubsystem<?>>... subsystems) {
        for (Class<? extends VLRSubsystem<?>> subsystem : subsystems) {
            if (!instances.containsKey(subsystem)) {
                try {
                    VLRSubsystem<?> instance = subsystem.getDeclaredConstructor().newInstance();
                    instances.put(subsystem, instance);
                } catch (Exception e) {
                    throw new RuntimeException("Error creating instance of " + subsystem.getName(), e);
                }
            }
        }
    }
}
