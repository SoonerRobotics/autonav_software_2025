"use client"

import { Label } from "@/components/ui/label"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import {
    Sidebar,
    SidebarContent,
    SidebarFooter,
    SidebarHeader,
    SidebarMenu,
    SidebarMenuButton,
    SidebarMenuItem,
    SidebarSeparator,
} from "@/components/ui/sidebar"
import { Switch } from "@/components/ui/switch"
import { useSocket } from "@/providers/SocketProvider"
import { Cog, LayoutDashboard, Map, MoreHorizontal, Settings } from "lucide-react"
import { useEffect, useState } from "react"

interface DashboardSidebarProps {
    activeScreen: string;
    onActiveScreenChange: (screen: string) => void;
    onCameraViewToggle?: () => void;
}

export function DashboardSidebar(props: DashboardSidebarProps) {
    const { lastMessage, api } = useSocket();
    const [systemState, setSystemState] = useState("0");
    const [mobility, setMobility] = useState(false);

    useEffect(() => {
        if (lastMessage?.type == "system_state")
        {
            setSystemState(`${lastMessage.data.state}`);
            setMobility(lastMessage.data.mobility);
        }
    }, [lastMessage]);

    const setMobilityExternal = (value: boolean) => {
        api.set_mobility(value);
    }

    const setSystemStateExternal = (value: string) => {
        api.set_system_state(value);
    }

    return (
        <Sidebar>
            <SidebarHeader className="flex items-center justify-center py-4">
                <h1 className="text-xl font-bold">Twistopher</h1>
            </SidebarHeader>
            <SidebarSeparator />
            <SidebarContent>
                <SidebarMenu>
                    <SidebarMenuItem>
                        <SidebarMenuButton isActive={props.activeScreen === "dashboard"} onClick={() => props.onActiveScreenChange("dashboard")}>
                            <LayoutDashboard />
                            <span>Dashboard</span>
                        </SidebarMenuButton>
                    </SidebarMenuItem>
                    <SidebarMenuItem>
                        <SidebarMenuButton isActive={props.activeScreen === "config"} onClick={() => props.onActiveScreenChange("config")}>
                            <Cog />
                            <span>Configuration</span>
                        </SidebarMenuButton>
                    </SidebarMenuItem>
                    <SidebarMenuItem>
                        <SidebarMenuButton isActive={props.activeScreen === "map"} onClick={() => props.onActiveScreenChange("map")}>
                            <Map />
                            <span>Map</span>
                        </SidebarMenuButton>
                    </SidebarMenuItem>
                    <SidebarMenuItem>
                        <SidebarMenuButton isActive={props.activeScreen === "swerve"} onClick={() => props.onActiveScreenChange("swerve")}>
                            <Settings />
                            <span>Swerve</span>
                        </SidebarMenuButton>
                    </SidebarMenuItem>
                    <SidebarMenuItem>
                        <SidebarMenuButton isActive={props.activeScreen === "other"} onClick={() => props.onActiveScreenChange("other")}>
                            <MoreHorizontal />
                            <span>Other</span>
                        </SidebarMenuButton>
                    </SidebarMenuItem>
                </SidebarMenu>
            </SidebarContent>
            <SidebarSeparator />
            <SidebarFooter className="p-4 space-y-4">
                <div className="space-y-2">
                    <Label htmlFor="system-state">System State</Label>
                    <Select value={systemState} onValueChange={(value) => {
                        setSystemStateExternal(value);
                    }}>
                        <SelectTrigger id="system-state">
                            <SelectValue placeholder="Select state" />
                        </SelectTrigger>
                        <SelectContent>
                            <SelectItem value={"0"}>Disabled</SelectItem>
                            <SelectItem value={"1"}>Manual</SelectItem>
                            <SelectItem value={"2"}>Autonomous</SelectItem>
                            <SelectItem value={"3"}>Shutdown</SelectItem>
                        </SelectContent>
                    </Select>
                </div>

                <div className="flex items-center justify-between">
                    <Label htmlFor="mobility">Mobility</Label>
                    <Switch id="mobility" checked={mobility} onCheckedChange={setMobilityExternal} />
                </div>

                <SidebarMenuButton onClick={props.onCameraViewToggle}>
                    <span>Toggle Camera View</span>
                </SidebarMenuButton>
            </SidebarFooter>
        </Sidebar>
    )
}
