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

        socket.on('connect', onConnect);
        socket.on('disconnect', onDisconnect);
        socket.on('motor_feedback', onMotorFeedback);
        socket.on('motor_input', onMotorInput);
        socket.on('gps_feedback', onGps);
        socket.on('system_state', onSystemState);
        socket.on('device_state', onDeviceState);

        return () => {
            socket.off('connect', onConnect);
            socket.off('disconnect', onDisconnect);
            socket.off('motor_feedback', onMotorFeedback);
            socket.off('motor_input', onMotorInput);
            socket.off('gps_feedback', onGps);
            socket.off('system_state', onSystemState);
            socket.off('device_state', onDeviceState);
        }
    }, []);

    if (!connected)
    {
        return <IdleScreen />
    }

    return (
        <div className="flex flex-row gap-4 p-8">
            <button onClick={() => {
                // prompt user for name
                const name = prompt("Enter preset name");
                if (name) {
                    socket.emit("save_preset", name);
                    toast.success(`Saved preset as ${name}`);
                } else {
                    toast.error("No name provided");
                }
            }}>
                Save Preset
            </button>
            
            <button onClick={() => {
                // prompt user for name
                const name = prompt("Enter preset name");
                if (name) {
                    socket.emit("load_preset", name);
                    toast.success(`Loaded preset ${name}`);
                } else {
                    toast.error("No name provided");
                }
            }}>   
                Load Preset
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
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
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
                                ({decimals(display.motor_feedback.delta_x, 2)}, {decimals(display.motor_feedback.delta_y, 2)}, {decimals(display.motor_feedback.delta_theta, 2)})
                            </div>
                        </div>
                    </div>

                    {/* display the latest motor input */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">Motor Input:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div className="text-lg">
                                ({decimals(display.motor_input.forward_velocity, 2)}, {decimals(display.motor_input.angular_velocity, 2)})
                            </div>
                        </div>
                    </div>

                    {/* display the latest gps */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">GPS:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div className="text-lg">
                                ({decimals(display.gps_feedback.latitude, 2)}, {decimals(display.gps_feedback.longitude, 2)}, {decimals(display.gps_feedback.altitude, 2)})
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            <div className="flex flex-col gap-4 ml-auto items-center justify-center">
                <div className="grid grid-cols-2 gap-4 w-fit">
                    <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/left_cam" />
                    <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/right_cam" />
                    <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/front_cam" />
                    <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/back_cam" />
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
