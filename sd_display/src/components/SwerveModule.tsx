import { motion } from "motion/react";

export function SwerveModule(props: {
    id: string;
    angle: number;
}) {
    const normalizedAngle = props.angle % 360;

    return (
        <div className="flex items-center justify-center w-full h-full">
            <div className="flex flex-col items-center justify-center p-4">
                <div className="text-xl font-bold mb-2">{props.id}</div>
                <div className="aspect-square w-full h-full rounded-full border-4 border-gray-300 flex items-center justify-center relative">
                    <motion.div
                        className="w-1 h-10 bg-blue-500"
                        animate={{ rotate: -normalizedAngle }}
                        transition={{ duration: 0.05 }}
                    />
                </div>
                <div className="mt-4 text-lg">{normalizedAngle.toFixed(1)}&deg;</div>
            </div>
        </div>
    );
};