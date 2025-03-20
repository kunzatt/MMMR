'use client';

import Image from 'next/image';
import { useState } from 'react';

const devices = [
    { id: 1, name: '전등', x: '20%', y: '30%' },
    { id: 2, name: '에어컨', x: '50%', y: '50%' },
    { id: 3, name: '스피커', x: '70%', y: '20%' },
    { id: 4, name: 'TV', x: '80%', y: '60%' },
];

export default function Iot() {
    const [activeDevices, setActiveDevices] = useState<number[]>([]);

    const toggleDevice = (id: number) => {
        setActiveDevices((prev) => (prev.includes(id) ? prev.filter((deviceId) => deviceId !== id) : [...prev, id]));
    };

    return (
        <div className="py-3 px-5 w-auto h-44 relative flex items-center justify-center overflow-hidden">
            <div className="relative w-full h-full flex items-center justify-center">
                <div className="relative w-48 h-36">
                    <Image
                        src="/images/iot_map.png"
                        alt="IoT Map"
                        fill
                        style={{ objectFit: 'contain' }}
                        className="rounded-md"
                    />

                    {devices.map((device) => (
                        <button
                            key={device.id}
                            onClick={() => toggleDevice(device.id)}
                            style={{ position: 'absolute', left: device.x, top: device.y }}
                            className={`absolute text-sm px-2 py-1 rounded-full shadow-md whitespace-nowrap
                                ${activeDevices.includes(device.id) ? 'bg-green-500 text-white' : 'bg-white'}
                            `}
                        >
                            {device.name}
                        </button>
                    ))}
                </div>
            </div>
        </div>
    );
}
