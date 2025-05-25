"use client"

import { DashboardContent } from "@/components/dashboard/dashboard-content"
import { DashboardSidebar } from "@/components/dashboard-sidebar"
import { SidebarProvider } from "@/components/ui/sidebar"
import { useState } from "react"
import { ConfigurationPage } from "../configuration/configuration-page"

export function RoboticsDashboard() {
    const [screen, setScreen] = useState("dashboard");

    return (
        <SidebarProvider>
            <div className="flex h-screen w-full bg-background max-h-screen">
                <DashboardSidebar activeScreen={screen} onActiveScreenChange={(new_screen) => {
                    setScreen(new_screen);
                }} />
                <div className="flex-1 overflow-auto">
                    {
                        screen === "dashboard" && (
                            <DashboardContent/>
                        )
                    }
                    {
                        screen === "config" && (
                            <ConfigurationPage />
                        )
                    }
                </div>
            </div>
        </SidebarProvider>
    )
}
