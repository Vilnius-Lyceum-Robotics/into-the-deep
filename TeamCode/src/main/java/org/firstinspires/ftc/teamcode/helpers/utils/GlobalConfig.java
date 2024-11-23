package org.firstinspires.ftc.teamcode.helpers.utils;

import com.acmerobotics.dashboard.config.Config;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

@Config
public class GlobalConfig {
    private static boolean DEBUG_MODE = false;

    private static final Map<ConfigVariable, Object> defaultValueMap = new HashMap<>();

    static {
        defaultValueMap.put(ConfigVariable.DEBUG_MODE, Boolean.FALSE);
    }

    @SuppressWarnings("unchecked")
    public static <T> T get(ConfigVariable variable) {
        try {
            Field field = GlobalConfig.class.getField(variable.name());
            Object value = field.get(null);
            if (value != null) {
                return (T) value;
            } else {
                return (T) defaultValueMap.get(variable);
            }
        } catch (NoSuchFieldException | IllegalAccessException e) {
            e.printStackTrace();
            return (T) defaultValueMap.get(variable);
        }
    }

    public enum ConfigVariable {
        DEBUG_MODE;
    }
}