"use client";

import IdleScreen from "@/components/IdleScreen";
import { SwerveModule } from "@/components/SwerveModule";
import { socket } from "@/lib/socket"; 
import { decimals, device_state_to_str, system_state_to_str } from "@/lib/text";
import { useEffect, useRef, useState } from "react";
import { toast } from "react-toastify";

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
    });
    const [connected, setConnected] = useState(false);

    useEffect(() => {
        const onConnect = () => {
            setConnected(true);
            toast.success("Connected to server");
        }

        const onDisconnect = () => {
            setConnected(false);
            toast.error("Disconnected from server");
        }

        const onMotorFeedback = (data: any) => {
            data = JSON.parse(data);
            setDisplay((prev) => ({
                ...prev,
                motor_feedback: {
                    delta_x: data.delta_x,
                    delta_y: data.delta_y,
                    delta_theta: data.delta_theta,
                },
            }));
        }

        const onMotorInput = (data: any) => {
            data = JSON.parse(data);
            setDisplay((prev) => ({
                ...prev,
                motor_input: {
                    forward_velocity: data.forward_velocity,
                    angular_velocity: data.angular_velocity,
                },
            }));
        }

        const onGps = (data: any) => {
            data = JSON.parse(data);
            setDisplay((prev) => ({
                ...prev,
                gps_feedback: {
                    latitude: data.latitude,
                    longitude: data.longitude,
                    altitude: data.altitude,
                },
            }));
        }

        const onSystemState = (data: any) => {
            data = JSON.parse(data);
            setDisplay((prev) => ({
                ...prev,
                system_state: data.state,
                is_mobility: data.is_mobility,
            }));
        }

        const onDeviceState = (data: any) => {
            data = JSON.parse(data);
            setDisplay((prev) => ({
                ...prev,
                device_states: {
                    ...prev.device_states,
                    [data.device]: data.state,
                },
            }));
        }

        const onPosition = (data: any) => {
            data = JSON.parse(data);
            setDisplay((prev) => ({
                ...prev,
                position: {
                    x: data.x,
                    y: data.y,
                    theta: data.theta,
                },
            }));
        }

        socket.on('connect', onConnect);
        socket.on('disconnect', onDisconnect);
        socket.on('motor_feedback', onMotorFeedback);
        socket.on('motor_input', onMotorInput);
        socket.on('gps_feedback', onGps);
        socket.on('system_state', onSystemState);
        socket.on('device_state', onDeviceState);
        socket.on('position', onPosition);

        return () => {
            socket.off('connect', onConnect);
            socket.off('disconnect', onDisconnect);
            socket.off('motor_feedback', onMotorFeedback);
            socket.off('motor_input', onMotorInput);
            socket.off('gps_feedback', onGps);
            socket.off('system_state', onSystemState);
            socket.off('device_state', onDeviceState);
            socket.off('position', onPosition);
        }
    }, []);

    if (!connected)
    {
        return <IdleScreen />
    }

    return (
        <div className="flex flex-row gap-4 p-8">
            <button onClick={() => {
                // prompt for a number and then send it 
                const state = prompt("Enter a system state (0-4):");
                if (state === null) {
                    return;
                }

                const state_num = parseInt(state);
                if (isNaN(state_num) || state_num < 0 || state_num > 4) {
                    toast.error("Invalid system state");
                    return;
                }

                socket.emit("set_system_state", state_num);
                toast.success(`Set system state to ${state_num}`);
            }}>
                set system state
            </button>

            {/* mobility, prompt fo true/false */}
            <button onClick={() => {
                const mobility = prompt("Is mobility enabled? (true/false):");
                if (mobility === null) {
                    return;
                }

                const mobility_bool = mobility.toLowerCase() === "true";
                socket.emit("set_mobility", mobility_bool);
                toast.success(`Set mobility to ${mobility_bool}`);
            }}>
                set mobility
            </button>
            
            <div className="flex flex-col gap-4">
                <div className="flex flex-col gap-2">
                    {/* display the system state */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">System State:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div className="text-lg">
                                {system_state_to_str(display.system_state)}
                            </div>
                        </div>
                    </div>

                    {/* dispaly all device states */}
                    <div className="flex flex-col gap-2">
                        <h2 className="text-xl font-bold">Device States:</h2>
                        <div className="overflow-auto h-full flex flex-col items-start ml-8 justify-center">
                            {Object.entries(display.device_states).map(([device, state]) => (
                                <div key={device} className="text-lg">
                                    {device}: {device_state_to_str(state)}
                                </div>
                            ))}
                        </div>
                    </div>

                    {/* display the latest motor feedback */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">Motor Feedback:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div className="text-lg">
                                ({decimals(display.motor_feedback.delta_x, 5)}, {decimals(display.motor_feedback.delta_y, 5)}, {decimals(display.motor_feedback.delta_theta, 5)})
                            </div>
                        </div>
                    </div>

                    {/* display the latest position */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">Position:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div className="text-lg">
                                ({decimals(display.position.x, 5)}, {decimals(display.position.y, 5)}, {decimals(display.position.theta, 5)})
                            </div>
                        </div>
                    </div>

                    {/* display the latest motor input */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">Motor Input:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div className="text-lg">
                                ({decimals(display.motor_input.forward_velocity, 4)}, {decimals(display.motor_input.angular_velocity, 4)})
                            </div>
                        </div>
                    </div>

                    {/* display the latest gps */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">GPS:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div className="text-lg">
                                ({decimals(display.gps_feedback.latitude, 7)}, {decimals(display.gps_feedback.longitude, 7)})
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            <div className="flex flex-col gap-4 ml-auto items-center justify-center">
                <div className="grid grid-cols-3 gap-4 w-fit">
                    <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/camera" />
                    <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/filtered" />
                    <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/expanded" />
                </div>
                <div className="grid grid-cols-2 gap-4 w-fit">
                    <SwerveModule id={"Front Left"} angle={0} />
                    <SwerveModule id={"Front Right"} angle={45} />
                    <SwerveModule id={"Bottom Left"} angle={95} />
                    <SwerveModule id={"Bottom Right"} angle={135} />
                </div>
            </div>
        </div>
    );
}
