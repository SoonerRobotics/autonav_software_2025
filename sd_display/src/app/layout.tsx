import type { Metadata } from "next";
import "./globals.css";

import ClientSide from "@/components/ClientSide";
import { SocketProvider } from "@/providers/SocketProvider";
import { ToastContainer } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

export const metadata: Metadata = {
    title: "SCR | Self Drive",
    description: "Sooner Competitive Robotics Self Drive User Interface",
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode; }>) {
    return (
        <html lang="en">
            <body className={`antialiased h-screen bg-gray-950 text-white`}>
                <SocketProvider>
                    {children}
                </SocketProvider>

                <ClientSide>
                    <ToastContainer theme="dark" pauseOnFocusLoss={false} pauseOnHover={false} />
                </ClientSide>
            </body>
        </html>
    );
}
