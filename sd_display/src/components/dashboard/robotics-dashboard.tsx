"use client"

import { DashboardContent } from "@/components/dashboard/dashboard-content"
import { DashboardSidebar } from "@/components/dashboard/dashboard-sidebar"
import { SidebarProvider } from "@/components/ui/sidebar"
import { useState } from "react"

export function RoboticsDashboard() {
    return (
        <SidebarProvider>
            <div className="flex h-screen w-full bg-background max-h-screen">
                <DashboardSidebar />
                <div className="flex-1 overflow-auto">
                    <DashboardContent/>
                </div>
            </div>
        </SidebarProvider>
    )
}
