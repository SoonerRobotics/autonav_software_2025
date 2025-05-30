"use client";

import { parse } from 'path';
import React, { createContext, ReactNode, useCallback, useContext, useEffect, useState } from 'react';
import { io, Socket } from 'socket.io-client';

interface SocketContextType {
    lastMessage: SocketMessage | null;
    api: {
        set_mobility: (value: boolean) => void;
        set_system_state: (value: string) => void;
        set_config: (device: string, cfg: any) => void;
    };
    configuration: any;
    deviceStates: Record<string, any>;
    setAddress: (ip: string, port: number) => void;
    getAddress: () => { ip: string; port: number };
    state: 'connected' | 'disconnected';
}

interface ServerToClientEvents {
    message: (message: any) => void;
}

interface ClientToServerEvents {
    set_mobility: (value: boolean) => void;
    set_system_state: (value: string) => void;
    set_config: (device: string, cfg: any) => void;
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
    const [configuration, setInternalConfig] = useState<any>({});
    const [deviceStates, setDeviceStates] = useState<Record<string, any>>({});
    const [address, setAddressState] = useState<{ ip: string; port: number }>({
        ip: '192.168.1.76',
        port: 4029,
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
            const msg = JSON.parse(message);
            const parsedMessage: SocketMessage = {
                type: msg.type,
                data: msg.msg,
            };
            
            if (parsedMessage.type == null)
            {
                return;
            }

            if (parsedMessage.type === "configs") {
                console.log(parsedMessage);
                // a map of device names -> configs (which are a map of key -> value)
                const configs = parsedMessage.data?.configs;
                setInternalConfig(configs);
            }

            if (parsedMessage.type === "device_state") {
                const { device, state } = parsedMessage.data;
                setDeviceStates((prevStates) => ({
                    ...prevStates,
                    [device]: state,
                }));
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
        set_mobility: (value: boolean) => {
            if (socket) {
                socket.emit('set_mobility', value);
            }
        },
        set_system_state: (value: string) => {
            if (socket) {
                socket.emit('set_system_state', value);
            }
        },
        set_config: (device: string, cfg: any) => {
            if (socket) {
                socket.emit('set_config', device, cfg);
            }
        }
    };

    const setAddress: SocketContextType['setAddress'] = (ip, port) => {
        setAddressState({ ip, port });
    };

    const getAddress: SocketContextType['getAddress'] = () => {
        return address;
    };

    return (
        <SocketContext.Provider value={{ lastMessage, api, setAddress, state, getAddress, configuration, deviceStates }}>
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
