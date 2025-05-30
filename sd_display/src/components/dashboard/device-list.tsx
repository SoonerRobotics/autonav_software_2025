import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"

export function DeviceList() {
    // Mock device data
    const devices = [
        { name: "Vision Combiner", node: "autonav_vision_combiner", status: 0 },
        { name: "Vision Transformer (Front)", node: "autonav_vision_transformer_front", status: 0 },
        { name: "Vision Transformer (Back)", node: "autonav_vision_transformer_back", status: 0 },
        { name: "Vision Transformer (Left)", node: "autonav_vision_transformer_left", status: 0 },
        { name: "Vision Transformer (Right)", node: "autonav_vision_transformer_right", status: 0 },
        { name: "Controller Interface", node: "controller_input", status: 0 },
        { name: "Manual Controller", node: "manual25_node", status: 0 },
        { name: "SparkMAX CAN Interface", node: "sparkmax_can_node", status: 0 },
        { name: "Twistopher CAN Interface", node: "CAN_node", status: 0 },
        { name: "Audible Feedback", node: "audible_feedback_node", status: 0 },
        { name: "Camera Node (Front)", node: "autonav_camera_front", status: 0 },
        { name: "Camera Node (Back)", node: "autonav_camera_back", status: 0 },
        { name: "Camera Node (Left)", node: "autonav_camera_left", status: 0 },
        { name: "Camera Node (Right)", node: "autonav_camera_right", status: 0 },
        { name: "Feelers", node: "autonav_camera_right", status: 0 },
    ]

    return (
        <Card>
            <CardHeader>
                <CardTitle>Nodes</CardTitle>
            </CardHeader>
            <CardContent>
                <div className="space-y-2 max-h-96 overflow-auto">
                    {devices.map((device, index) => (
                        <div key={index} className="flex items-start justify-between py-1 border-b last:border-0">
                            <span className="text-sm">{device.name}</span>
                            <Badge
                                variant={
                                    device.status === 0 ? "outline" : device.status === 1 ? "secondary" : "destructive"
                                }
                            >
                                {device.status == 0 ? "Off" : device.status == 1 ? "Warming" : device.status == 2 ? "Ready" : device.status == 3 ? "Operating" : "Unknown"}
                            </Badge>
                        </div>
                    ))}
                </div>
            </CardContent>
        </Card>
    )
}
