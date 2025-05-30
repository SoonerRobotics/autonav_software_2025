import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { useSocket } from "@/providers/SocketProvider"
import { useEffect, useState } from "react"

export function SystemStatus() {
    const [gpsData, setGpsData] = useState({
        latitude: 0,
        longitude: 0,
    })
    const [positionData, setPositionData] = useState({
        x: 0,
        y: 0,
        theta: 0,
    })
    const [motorFeedback, setMotorFeedback] = useState({
        delta_x: 0,
        delta_y: 0,
        delta_theta: 0,
    })
    const [motorInput, setMotorInput] = useState({
        forward_velocity: 0,
        sideways_velocity: 0,
        angular_velocity: 0,
    })
    const { lastMessage } = useSocket();

    useEffect(() => {
        if (lastMessage == null)
        {
            return;
        }

        if (lastMessage.type == "motor_feedback")
        {
            setMotorFeedback(lastMessage.data);
        }
        if (lastMessage.type == "motor_input")
        {
            setMotorInput(lastMessage.data);
        }
        if (lastMessage.type == "gps")
        {
            setGpsData(lastMessage.data);
        }
        if (lastMessage.type == "position")
        {
            setPositionData(lastMessage.data);
        }
    }, [lastMessage]);

    return (
        <Card>
            <CardHeader>
                <CardTitle>System Status</CardTitle>
            </CardHeader>
            <CardContent className="space-y-4">
                <div>
                    <h3 className="text-sm font-medium mb-2">Current Position</h3>
                    <div className="grid grid-cols-3 gap-2 text-sm">
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">X</div>
                            <div>{positionData.x.toFixed(2)} m</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Y</div>
                            <div>{positionData.y.toFixed(2)} m</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">θ</div>
                            <div>{positionData.theta.toFixed(2)} rad</div>
                        </div>
                    </div>
                </div>

                <div>
                    <h3 className="text-sm font-medium mb-2">GPS Location</h3>
                    <div className="grid grid-cols-2 gap-2 text-sm">
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Latitude</div>
                            <div>{gpsData.latitude.toFixed(4)}°</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Longitude</div>
                            <div>{gpsData.longitude.toFixed(4)}°</div>
                        </div>
                    </div>
                </div>

                <div>
                    <h3 className="text-sm font-medium mb-2">Motor Feedback</h3>
                    <div className="grid grid-cols-3 gap-2 text-sm">
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Δx</div>
                            <div>{motorFeedback.delta_x.toFixed(2)} m</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Δy</div>
                            <div>{motorFeedback.delta_y.toFixed(2)} m</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Δθ</div>
                            <div>{motorFeedback.delta_theta.toFixed(2)} rad</div>
                        </div>
                    </div>
                </div>

                <div>
                    <h3 className="text-sm font-medium mb-2">Motor Input</h3>
                    <div className="grid grid-cols-3 gap-2 text-sm">
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Forward</div>
                            <div>{motorInput.forward_velocity.toFixed(2)}</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Sideways</div>
                            <div>{motorInput.sideways_velocity.toFixed(2)}</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Angular</div>
                            <div>{motorInput.angular_velocity.toFixed(2)}</div>
                        </div>
                    </div>
                </div>
            </CardContent>
        </Card>
    )
}
