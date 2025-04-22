"use client";

import IdleScreen from "@/components/IdleScreen";
import { socket } from "@/lib/socket"; 
import { useEffect, useRef, useState } from "react";
import { toast } from "react-toastify";

export default function Home() {
    const motorFeedbackRef = useRef<HTMLDivElement>(null);
    const motorInputRef = useRef<HTMLDivElement>(null);
    const gpsRef = useRef<HTMLDivElement>(null);
    const [render, setRender] = useState(false);

    useEffect(() => {
        const onConnect = () => {
            setRender(!render);
            toast.success("Connected to server");
        }

        const onDisconnect = () => {
            setRender(!render);
            toast.error("Disconnected from server");
        }

        // function onMessage(data: any) {
        //     if (motorFeedbackRef.current) {
        //         motorFeedbackRef.current.innerHTML = JSON.stringify(data, null, 2);
        //     }
        // }

        const onMotorFeedback = (data: any) => {
            if (motorFeedbackRef.current) {
                data = JSON.parse(data);
                motorFeedbackRef.current.innerHTML = `(${data.delta_x}, ${data.delta_y}, ${data.delta_theta})`;
            }
        }

        const onMotorInput = (data: any) => {
            if (motorInputRef.current) {
                motorInputRef.current.innerHTML = JSON.stringify(data, null, 2);
            }
        }

        const onGps = (data: any) => {
            if (gpsRef.current) {
                gpsRef.current.innerHTML = JSON.stringify(data, null, 2);
            }
        }

        socket.on('connect', onConnect);
        socket.on('disconnect', onDisconnect);
        socket.on('motor_feedback', onMotorFeedback);
        socket.on('motor_input', onMotorInput);
        socket.on('gps_feedback', onGps);

        return () => {
            socket.off('connect', onConnect);
            socket.off('disconnect', onDisconnect);
            socket.off('motor_feedback', onMotorFeedback);
            socket.off('motor_input', onMotorInput);
            socket.off('gps_feedback', onGps);
        }
    }, []);

    if (!socket.connected)
    {
        return <IdleScreen />
    }

    return (
        <div className="flex flex-row gap-4 p-8">
            <div className="flex flex-col gap-4">
                <div className="flex flex-col gap-2">
                    {/* display the latest motor feedback */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">Motor Feedback:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div ref={motorFeedbackRef} className="text-lg" />
                        </div>
                    </div>

                    {/* display the latest motor input */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">Motor Input:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div ref={motorInputRef} className="text-lg" />
                        </div>
                    </div>

                    {/* display the latest gps */}
                    <div className="flex flex-row gap-2">
                        <h2 className="text-center text-xl font-bold">GPS:</h2>
                        <div className="overflow-auto h-full flex flex-col items-center justify-center">
                            <div ref={gpsRef} className="text-lg" />
                        </div>
                    </div>
                </div>
            </div>
            <div className="grid grid-cols-2 gap-4 w-fit ml-auto">
                <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/left_cam" />
                <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/right_cam" />
                <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/front_cam" />
                <img className="w-[200px] h-[200px] bg-slate-500 rounded-lg p-1" src="http://localhost:4029/back_cam" />
            </div>

        </div>
    );
}
