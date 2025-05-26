"use client"

import { DashboardContent } from "@/components/dashboard/dashboard-content"
import { DashboardSidebar } from "@/components/dashboard-sidebar"
import { SidebarProvider } from "@/components/ui/sidebar"
import { useEffect, useState } from "react"
import { ConfigurationPage } from "../configuration/configuration-page"
import { CameraViewPopup } from "../camera-view-popup"

export function RoboticsDashboard() {
    const [screen, setScreen] = useState("dashboard");
    const [isPopupOpen, setIsPopupOpen] = useState(false);

    useEffect(() => {
        const handleKeyPress = (event: KeyboardEvent) => {
            if (event.ctrlKey && event.shiftKey && event.key === "C") {
                event.preventDefault()
                setIsPopupOpen((prev) => !prev)
            }
        }

        document.addEventListener("keydown", handleKeyPress)
        return () => document.removeEventListener("keydown", handleKeyPress)
    }, [])

    return (
        <SidebarProvider>
            <div className="flex h-screen w-full bg-background max-h-screen">
                <DashboardSidebar activeScreen={screen} onActiveScreenChange={(new_screen) => {
                    setScreen(new_screen);
                }} />
                <div className="flex-1 overflow-auto">
                    {
                        screen === "dashboard" && (
                            <DashboardContent />
                        )
                    }
                    {
                        screen === "config" && (
                            <ConfigurationPage />
                        )
                    }
                </div>

                <CameraViewPopup isOpen={isPopupOpen} onClose={() => setIsPopupOpen(false)} />
            </div>
        </SidebarProvider>
    )
}
