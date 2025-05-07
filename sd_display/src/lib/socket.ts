import { io } from 'socket.io-client';

export const socket = io('http://localhost:4029', {
    autoConnect: true,
    reconnection: true
});