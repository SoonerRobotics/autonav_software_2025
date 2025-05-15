export function decimals(value: number, decimalPlaces: number) {
    return Number(value).toLocaleString(undefined, {
        minimumFractionDigits: decimalPlaces,
        maximumFractionDigits: decimalPlaces,
    });
}

export function device_state_to_str(value: number) {
    switch (value)
    {
        case 0: return "OFF";
        case 1: return "WARMING";
        case 2: return "READY";
        case 3: return "OPERATING";
        case 4: return "ERROR";
        default: return "UNKNOWN";
    }
}

export function system_state_to_str(value: number) {
    switch (value)
    {
        case 0: return "DISABLED";
        case 1: return "MANUAL";
        case 2: return "AUTONOMOUS";
        case 3: return "SHUTDOWN";
        default: return "UNKNOWN";
    }
}