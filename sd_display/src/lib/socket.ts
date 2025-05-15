import { io } from 'socket.io-client';

export const socket = io('http://192.168.1.76:4029', {
    autoConnect: true,
    reconnection: true
});