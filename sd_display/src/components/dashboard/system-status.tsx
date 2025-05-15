import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"

interface SystemStatusProps {
    systemState: string
    mobility: boolean
}

export function SystemStatus({ systemState, mobility }: SystemStatusProps) {
    // Mock GPS data
    const gpsData = {
        latitude: 37.7749,
        longitude: -122.4194,
    }

    // Mock position data
    const positionData = {
        x: 2.45,
        y: 1.32,
        theta: 0.78,
    }

    // Mock motor feedback data
    const motorFeedback = {
        delta_x: 0.1,
        delta_y: -0.05,
        delta_theta: 0.02,
    }

    // Mock motor input data
    const motorInput = {
        forward: 0.5,
        sideways: -0.2,
        angular: 0.1,
    }

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
                            <div>{motorInput.forward.toFixed(2)}</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Sideways</div>
                            <div>{motorInput.sideways.toFixed(2)}</div>
                        </div>
                        <div className="bg-muted p-2 rounded-md">
                            <div className="text-xs text-muted-foreground">Angular</div>
                            <div>{motorInput.angular.toFixed(2)}</div>
                        </div>
                    </div>
                </div>
            </CardContent>
        </Card>
    )
}
