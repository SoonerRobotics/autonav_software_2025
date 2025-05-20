"use client"

import { DeviceList } from "@/components/dashboard/device-list"
import { IndividualCameraGrid } from "@/components/dashboard/individual-camera-grid"
import { SwerveVisualization } from "@/components/dashboard/swerve-visualization"
import { SystemStatus } from "@/components/dashboard/system-status"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { CombinedCameraGrid } from "./combined-camera-grid"

export function DashboardContent() {
    return (
        <div className="flex flex-col h-full p-6 space-y-6">
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                <SystemStatus />
                <Card>
                    <CardHeader>
                        <CardTitle>Camera Feeds</CardTitle>
                    </CardHeader>
                    <CardContent className="h-[300px]">
                        <IndividualCameraGrid />
                    </CardContent>
                </Card>
                <DeviceList />
            </div>

            <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 flex-grow min-h-0">
                <Card className="lg:col-span-1">
                    <CardHeader>
                        <CardTitle>Swerve Visualization</CardTitle>
                    </CardHeader>
                    <CardContent className="h-[300px]">
                        <SwerveVisualization />
                    </CardContent>
                </Card>

                <Card className="lg:col-span-2 flex flex-col">
                    <CardHeader className="pb-2">
                        <CardTitle>Camera Feeds</CardTitle>
                    </CardHeader>
                    <CardContent className="">
                        <CombinedCameraGrid />
                    </CardContent>
                </Card>
            </div>
        </div>
    )
}
