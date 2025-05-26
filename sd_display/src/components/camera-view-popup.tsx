"use client"

import { useState, useEffect } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { X, Camera, Eye, Route } from "lucide-react"
import { useSocket } from "@/providers/SocketProvider"

interface CameraViewPopupProps {
    isOpen: boolean
    onClose: () => void
}

type CameraView = "regular" | "transformed" | "astar"

export function CameraViewPopup({ isOpen, onClose }: CameraViewPopupProps) {
    const [selectedView, setSelectedView] = useState<CameraView>("regular")
    const { getAddress } = useSocket();

    const cameraViews = [
        {
            id: "regular" as CameraView,
            name: "Regular",
            path: "camera_front",
            icon: Camera,
        },
        {
            id: "transformed" as CameraView,
            name: "Transformed",
            path: "filtered",
            icon: Eye,
        },
        {
            id: "astar" as CameraView,
            name: "A*",
            path: "expanded",
            icon: Route,
        },
    ]

    if (!isOpen) return null

    return (
        <>
            {/* Popup Panel */}
            <div className="fixed bottom-4 right-4 z-50 w-80">
                <Card className="shadow-lg border-2">
                    <CardHeader className="pb-3">
                        <div className="flex items-center justify-between">
                            <CardTitle className="text-base">Camera View</CardTitle>
                            <Button variant="ghost" size="sm" onClick={onClose}>
                                <X className="h-4 w-4" />
                            </Button>
                        </div>
                    </CardHeader>
                    <CardContent className="space-y-2">
                        {cameraViews.map((view, index) => {
                            const Icon = view.icon
                            const isSelected = selectedView === view.id

                            return (
                                <div
                                    key={view.id}
                                    className={`p-3 rounded-lg border-2 cursor-pointer transition-all ${isSelected
                                            ? "border-primary bg-primary/10"
                                            : "border-border hover:border-primary/50 hover:bg-muted/50"
                                        }`}
                                    onClick={() => setSelectedView(view.id)}
                                >
                                    <div className="flex items-center gap-3">
                                        <div className={`p-2 rounded-md ${isSelected ? "bg-primary text-primary-foreground" : "bg-muted"}`}>
                                            <Icon className="h-4 w-4" />
                                        </div>
                                        <div className="flex-1">
                                            <div className="flex items-center gap-2">
                                                <span className="font-medium text-sm">{view.name}</span>
                                                <Badge variant="outline" className="text-xs">
                                                    {index + 1}
                                                </Badge>
                                            </div>
                                        </div>
                                        {isSelected && <div className="w-2 h-2 bg-primary rounded-full" />}
                                    </div>
                                </div>
                            )
                        })}

                        <div className="pt-2 border-t">
                            <div className="flex items-center justify-between text-xs text-muted-foreground">
                                <span>Current View:</span>
                                <Badge variant="secondary">{cameraViews.find((v) => v.id === selectedView)?.name}</Badge>
                            </div>
                        </div>

                        {/* Mock camera preview */}
                        <div className="mt-4">
                            <img className="bg-slate-500 rounded-lg w-full h-full" src={`http://${getAddress().ip}:${getAddress().port}/${cameraViews.find((v) => v.id === selectedView)?.path}`} />
                        </div>
                    </CardContent>
                </Card>
            </div>
        </>
    )
}
