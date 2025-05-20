"use client";

import { RoboticsDashboard } from "@/components/dashboard/robotics-dashboard";
import IdleScreen from "@/components/IdleScreen";
import { useSocket } from "@/providers/SocketProvider";
import { useEffect } from "react";

export default function Home() {
    const { state, getAddress } = useSocket();

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
