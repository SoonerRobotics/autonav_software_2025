"use client"

import { DashboardContent } from "@/components/dashboard/dashboard-content"
import { DashboardSidebar } from "@/components/dashboard/dashboard-sidebar"
import { SidebarProvider } from "@/components/ui/sidebar"
import { useState } from "react"

export function RoboticsDashboard() {
    const [systemState, setSystemState] = useState<"Disabled" | "Manual" | "Autonomous" | "Shutdown">("Disabled")
    const [mobility, setMobility] = useState<boolean>(false)

    return (
        <SidebarProvider>
            <div className="flex h-screen w-full bg-background max-h-screen">
                <DashboardSidebar
                    systemState={systemState}
                    setSystemState={setSystemState}
                    mobility={mobility}
                    setMobility={setMobility}
                />
                <div className="flex-1 overflow-auto">
                    <DashboardContent systemState={systemState} mobility={mobility} />
                </div>
            </div>
        </SidebarProvider>
    )
}
