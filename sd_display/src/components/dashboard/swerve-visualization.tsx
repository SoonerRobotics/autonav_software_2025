import { useEffect, useState } from "react";
import { SwerveModule } from "../SwerveModule";
import { number } from "motion/react";
import { useSocket } from "@/providers/SocketProvider";

export function SwerveVisualization() {
    const [ swerveState, setSwerveState ] = useState({
        position_fl: 0,
        position_fr: 0,
        position_bl: 0,
        position_br: 0,
    })
    const { lastMessage } = useSocket();

    useEffect(() => {
        if (lastMessage == null)
        {
            return;
        }

        if (lastMessage.type == "swerve_state")
        {
            setSwerveState(lastMessage.data);
        }
    }, [lastMessage]);

    return (
        <div className="flex items-center justify-center rounded-md">
            <div className="grid grid-cols-2 w-full h-full">
                <SwerveModule id={"Front Left"} angle={swerveState.position_fl} />
                <SwerveModule id={"Front Right"} angle={swerveState.position_fr} />
                <SwerveModule id={"Back Left"} angle={swerveState.position_bl} />
                <SwerveModule id={"Back Right"} angle={swerveState.position_br} />
            </div>
        </div>
    )
}
