"use client";

import { RoboticsDashboard } from "@/components/dashboard/robotics-dashboard";
import IdleScreen from "@/components/IdleScreen";
import { useSocket } from "@/providers/SocketProvider";
import { useEffect, useState } from "react";

type DisplayData = {
    system_state: number;
    is_mobility: boolean;
    device_states: {
        [key: string]: number;
    };
    motor_feedback: {
        delta_x: number;
        delta_y: number;
        delta_theta: number;
    };
    motor_input: {
        forward_velocity: number;
        sideways_velocity: number;
        angular_velocity: number;
    };
    position: {
        x: number;
        y: number;
        theta: number;
    },
    gps_feedback: {
        latitude: number;
        longitude: number;
        altitude: number;
    },
    absolute_positions: {
        position_fl: number;
        position_fr: number;
        position_bl: number;
        position_br: number;
    };
}

export default function Home() {
    const [display, setDisplay] = useState<DisplayData>({
        system_state: 0,
        is_mobility: false,
        device_states: {},
        motor_feedback: {
            delta_x: 0,
            delta_y: 0,
            delta_theta: 0,
        },
        motor_input: {
            forward_velocity: 0,
            sideways_velocity: 0,
            angular_velocity: 0,
        },
        position: {
            x: 0,
            y: 0,
            theta: 0,
        },
        gps_feedback: {
            latitude: 0,
            longitude: 0,
            altitude: 0,
        },
        absolute_positions: {
            position_fl: 0,
            position_fr: 0,
            position_bl: 0,
            position_br: 0,
        },
    });
    const { state, api, lastMessage, getAddress } = useSocket();

    useEffect(() => {
        if (lastMessage == null)
        {
            return;
        }

        console.log(lastMessage.type);
    }, [lastMessage]);

    useEffect(() => {
        if (state !== "connected")
        {
            return;
        }

        // Update local storage on successful connection
        localStorage.setItem("host", getAddress().ip);
        localStorage.setItem("port", getAddress().port.toString());
    }, [state]);

    if (state !== "connected") {
        return <IdleScreen />
    }

    return (
        <RoboticsDashboard />
    );
}
