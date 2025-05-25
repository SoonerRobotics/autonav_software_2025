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
import { Cog, Eye, LayoutDashboard, MoreHorizontal, Settings } from "lucide-react"
import { useEffect, useState } from "react"

export function DashboardSidebar() {
    const { lastMessage } = useSocket();
    const [systemState, setSystemState] = useState("0");
    const [mobility, setMobility] = useState(false);

    useEffect(() => {
        if (lastMessage?.type == "system_state")
        {
            setSystemState(`${lastMessage.data.state}`);
            setMobility(lastMessage.data.mobility);
        }
    }, [lastMessage]);

    return (
        <Sidebar>
            <SidebarHeader className="flex items-center justify-center py-4">
                <h1 className="text-xl font-bold">Twistopher</h1>
            </SidebarHeader>
            <SidebarSeparator />
            <SidebarContent>
                <SidebarMenu>
                    <SidebarMenuItem>
                        <SidebarMenuButton isActive>
                            <LayoutDashboard />
                            <span>Dashboard</span>
                        </SidebarMenuButton>
                    </SidebarMenuItem>
                    <SidebarMenuItem>
                        <SidebarMenuButton>
                            <Settings />
                            <span>Swerve</span>
                        </SidebarMenuButton>
                    </SidebarMenuItem>
                    <SidebarMenuItem>
                        <SidebarMenuButton>
                            <Eye />
                            <span>Vision</span>
                        </SidebarMenuButton>
                    </SidebarMenuItem>
                    <SidebarMenuItem>
                        <SidebarMenuButton>
                            <Cog />
                            <span>Configuration</span>
                        </SidebarMenuButton>
                    </SidebarMenuItem>
                    <SidebarMenuItem>
                        <SidebarMenuButton>
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
                    <Select value={systemState} onValueChange={(value) => setSystemState(value as any)}>
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
                    <Switch id="mobility" checked={mobility} onCheckedChange={setMobility} />
                </div>
            </SidebarFooter>
        </Sidebar>
    )
}
