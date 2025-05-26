import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { useSocket } from "@/providers/SocketProvider"

const devices = [
    { name: "Vision Transformer", node: "autonav_vision_transformer" },
    { name: "Vision Expandifier", node: "autonav_vision_expandifier" },
    { name: "Filters", node: "zemlin_filters" },
    { name: "Display Backend", node: "autonav_sd_display" },
    { name: "AStar", node: "autonav_nav_astar" },
    { name: "Controller Interface", node: "autonav_controller_input" },
    { name: "Manual Controller", node: "autonav_manual" },
    { name: "SparkMAX CAN Interface", node: "sparkmax_can_node" },
    { name: "Twistopher CAN Interface", node: "autonav_can" },
    { name: "Audible Feedback", node: "audible_feedback_node" },
    { name: "Camera Node", node: "autonav_camera_front" },
]

const stateMap = {
    0: {
        label: "Off",
        color: "#131313",
    },
    1: {
        label: "Warming",
        color: "#f59e0b",
    },
    2: {
        label: "Ready",
        color: "#22c55e",
    },
    3: {
        label: "Operating",
        color: "#2563eb",
    },
}

export function DeviceList() {
    const { deviceStates } = useSocket();

    return (
        <Card>
            <CardHeader>
                <CardTitle>Nodes</CardTitle>
            </CardHeader>
            <CardContent>
                <div className="space-y-2 overflow-auto">
                    {devices.map((device, index) => {
                        const state = deviceStates[device.node] || 0; // Default to 0 if not found
                        const name = device.name || device.node;
                        return (<div key={index} className="flex items-start justify-between py-1 border-b last:border-0">
                            <span className="text-sm">{name}</span>
                            <Badge
                                variant="default"
                                style={{
                                    backgroundColor: (stateMap as any)[state]?.color || "#e5e7eb",
                                    color: "#ffffff"
                                }}
                            >
                                {(stateMap as any)[state]?.label || "Unknown"}
                            </Badge>
                        </div>);
                    })}
                </div>
            </CardContent>
        </Card>
    )
}
