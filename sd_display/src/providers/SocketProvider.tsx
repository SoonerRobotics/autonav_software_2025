"use client";

import React, { createContext, ReactNode, useCallback, useContext, useEffect, useState } from 'react';
import { io, Socket } from 'socket.io-client';

interface SocketContextType {
    lastMessage: SocketMessage | null;
    api: {
        ping: () => void;
    };
    setAddress: (ip: string, port: number) => void;
    getAddress: () => { ip: string; port: number };
    state: 'connected' | 'disconnected';
}

interface ServerToClientEvents {
    message: (message: any) => void;
}

interface ClientToServerEvents {
    ping: () => void;
}

const SocketContext = createContext<SocketContextType | undefined>(undefined);

interface SocketProviderProps {
    children: ReactNode;
}

interface SocketMessage {
    type: string;
    data: any;
}

const DEFAULT_TIMEOUT = 3000;

export const SocketProvider: React.FC<SocketProviderProps> = ({ children }) => {
    const [socket, setSocket] = useState<Socket<ServerToClientEvents, ClientToServerEvents> | null>(null);
    const [lastMessage, setLastMessage] = useState<SocketMessage | null>(null);
    const [state, setState] = useState<'connected' | 'disconnected'>('disconnected');
    const [address, setAddressState] = useState<{ ip: string; port: number }>({
        ip: 'localhost',
        port: 3000,
    });

    const connectSocket = useCallback(() => {
        if (socket) {
            socket.disconnect();
        }

        const newSocket: Socket<ServerToClientEvents, ClientToServerEvents> = io(`http://${address.ip}:${address.port}`, {
            timeout: DEFAULT_TIMEOUT,
            reconnection: true,
            reconnectionAttempts: Infinity
        });
        
        newSocket.on('connect', () => {
            console.log('Connected to socket server');
            setState('connected');
        });

        newSocket.on('disconnect', (reason: string) => {
            console.warn('Disconnected:', reason);
            setState('disconnected');
        });

        newSocket.on('message', (message: any) => {
            const parsedMessage: SocketMessage = {
                type: message.type,
                data: message.data,
            };

            if (parsedMessage.type == null)
            {
                return;
            }

            setLastMessage(parsedMessage);
        });

        setSocket(newSocket);
    }, [address]);

    useEffect(() => {
        connectSocket();

        return () => {
            socket?.disconnect();
        };
    }, [connectSocket]);

    const api: SocketContextType['api'] = {
        ping: () => {
            if (socket && socket.connected) {
                socket.emit('ping');
            } else {
                console.warn('Socket is not connected');
            }
        },
    };

    const setAddress: SocketContextType['setAddress'] = (ip, port) => {
        setAddressState({ ip, port });
    };

    const getAddress: SocketContextType['getAddress'] = () => {
        return address;
    };

    return (
        <SocketContext.Provider value={{ lastMessage, api, setAddress, state, getAddress }}>
            {children}
        </SocketContext.Provider>
    );
};

export const useSocket = (): SocketContextType => {
    const context = useContext(SocketContext);
    if (!context) {
        throw new Error('useSocket must be used within a SocketProvider');
    }
    return context;
};
