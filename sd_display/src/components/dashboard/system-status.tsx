import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { useSocket } from "@/providers/SocketProvider"
import { useEffect, useState } from "react"

export function SystemStatus() {
    const [gpsData, setGpsData] = useState({
        latitude: 0,
        longitude: 0,
        num_satellites: 0,
        gps_fix: 0,
    })
    const [positionData, setPositionData] = useState({
        x: 0,
        y: 0,
        theta: 0,
        latitude: 0,
        longitude: 0,
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
    const [pathing, setPathing] = useState({
        desired_latitude: 0,
        desired_longitude: 0,
        desired_heading: 0,
        distance_to_destination: 0,
        waypoints: [],
        time_until_use_waypoints: 0,
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
        if (lastMessage.type == "gps_feedback")
        {
            setGpsData(lastMessage.data);
        }
        if (lastMessage.type == "position")
        {
            setPositionData(lastMessage.data);
        }
        if (lastMessage.type == "pathing_debug")
        {
            setPathing(lastMessage.data);
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
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Latitude</div>
                            <div>{positionData.latitude.toFixed(7)}°</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md w-full">
                            <div className="text-xs text-muted-foreground">Longitude</div>
                            <div>{positionData.longitude.toFixed(7)}°</div>
                        </div>
                    </div>
                </div>

                <div>
                    <h3 className="text-sm font-medium mb-2">GPS Location</h3>
                    <div className="grid grid-cols-2 gap-2 text-sm">
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Latitude</div>
                            <div>{gpsData.latitude.toFixed(7)}°</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Longitude</div>
                            <div>{gpsData.longitude.toFixed(7)}°</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Satellites</div>
                            <div>{gpsData.num_satellites}</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">GPS Fix</div>
                            <div>{gpsData.gps_fix}</div>
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

                <div>
                    <h3 className="text-sm font-medium mb-2">Pathing</h3>
                    <div className="grid grid-cols-3 gap-2 text-sm">
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Desired Latitude</div>
                            <div>{pathing.desired_latitude.toFixed(4)}°</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Desired Longitude</div>
                            <div>{pathing.desired_longitude.toFixed(4)}°</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Desired Heading</div>
                            <div>{pathing.desired_heading.toFixed(2)} rad</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Distance to Destination</div>
                            <div>{pathing.distance_to_destination.toFixed(2)} m</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Time Until Use Waypoints</div>
                            <div>{pathing.time_until_use_waypoints.toFixed(2)} s</div>
                        </div>
                    </div>

                    <div className="bg-muted p-2 rounded-md">
                        <div className="text-xs text-muted-foreground">Waypoints</div>
                        <div>{pathing.waypoints.length} waypoints</div>
                        <div className="text-xs text-muted-foreground">[</div>
                        {/* {pathing.waypoints.map((waypoint: any, index) => (
                            <div key={index} className="text-xs text-muted-foreground">
                                {`(${waypoint.latitude.toFixed(4)}, ${waypoint.longitude.toFixed(4)})`}
                                {index < pathing.waypoints.length - 1 ? ', ' : ''}
                            </div>
                        ))}
                        <div className="text-xs text-muted-foreground">]</div> */}
                    </div>
                </div>
            </CardContent>
        </Card>
    )
}
