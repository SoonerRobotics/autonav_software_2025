import { useSocket } from "@/providers/SocketProvider";

export function CombinedCameraGrid() {
    return (
        <div className="space-y-6">
            <div className="grid grid-cols-2 gap-4 max-h-full">
                <CameraFeed name="Filtered" path="filtered" />
                <CameraFeed name="Feelered" path="expanded" />
            </div>
        </div>
    )
}

interface CameraFeedProps {
    name: string
    path: string
    aspectRatio?: string
}

function CameraFeed({ name, path, aspectRatio = "8/4" }: CameraFeedProps) {
    const { getAddress } = useSocket();

    return (
        <div className={`space-y-1`}>
            <div className="text-xs font-medium">{name}</div>
            <div className="bg-muted/50 rounded-md border flex items-center justify-center" style={{ aspectRatio }}>
                {/* <div className="text-xs text-muted-foreground">{name}</div> */}
                {/* <div className="text-[10px] text-muted-foreground mt-1">Camera feed placeholder</div> */}
                <img className=" bg-slate-500 rounded-lg w-full h-full" src={`http://${getAddress().ip}:${getAddress().port}/${path}`} />
            </div>
        </div>
    )
}
