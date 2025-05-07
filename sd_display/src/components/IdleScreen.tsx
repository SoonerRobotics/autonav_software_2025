import { Oval } from "react-loading-icons";

export default function IdleScreen()
{
    return (
        <div className="flex items-center justify-center h-full w-full" suppressHydrationWarning>
            {/* <Oval stroke="white" fill="none" width="48" height="48" /> */}
            <div suppressHydrationWarning className="ml-4 text-4xl">Waiting for a robot to connect</div>
        </div>
    )
}