import { useSocket } from "@/providers/SocketProvider";
import { Loader2 } from "lucide-react";
import { useEffect, useState } from "react";
import { Button } from "./ui/button";
import { Input } from "./ui/input";

export default function IdleScreen()
{
    const { setAddress } = useSocket();
    const [host, setHost] = useState<string>("");
    const [port, setPort] = useState<number>(0);

    useEffect(() => {
        const host = localStorage.getItem("host");
        const port = localStorage.getItem("port");

        if (host && port) {
            setHost(host);
            setPort(parseInt(port));
            setAddress(host, parseInt(port));
        }
    }, []);

    console.log(host, port);

    return (
        <div className="flex flex-col items-center justify-center h-full w-full gap-4" suppressHydrationWarning>
            <div className="flex flex-row gap-4 justify-center items-center">
                <Loader2 className="animate-spin w-8 h-8" />
                <div suppressHydrationWarning className="text-4xl">Waiting for Twistopher</div>
            </div>

            <div className="flex flex-col gap-2 p-4 z-40">
                <div className="flex flex-row gap-2 items-center justify-center h-fit">
                    <Input value={host} onChange={(e) => setHost(e.target.value)} placeholder="Host" className="w-64" />
                    <Input value={port} onChange={(e) => setPort(parseInt(e.target.value))} placeholder="Port" type="number" className="w-32 [appearance:textfield] [&::-webkit-outer-spin-button]:appearance-none [&::-webkit-inner-spin-button]:appearance-none" />
                </div>

                <Button
                    onClick={() => { setAddress(host, port); }
                    }
                    className="w-32"
                    variant="secondary"
                    disabled={host.length === 0 || port <= 0}
                >
                    Save
                </Button>
            </div>
        </div>
    )
}