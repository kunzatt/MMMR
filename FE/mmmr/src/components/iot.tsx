"use client";

import API_ROUTES from "@/config/apiRoutes";
import { getToken } from "@/config/getToken";
import Image from "next/image";
import { useEffect, useState } from "react";
import { IoInformationCircleOutline } from "react-icons/io5";

const devicesPos = [
    { device: "livingroomLight", x: "43%", y: "20%" },
    { device: "TV", x: "54%", y: "17%" },
    { device: "airConditioner", x: "50%", y: "5%" },
    { device: "airPurifier", x: "30%", y: "40%" },
    { device: "curtain", x: "42%", y: "5%" },
    { device: "kitchenLight", x: "46%", y: "75%" },
    { device: "entranceLight", x: "68%", y: "70%" }
];

interface Device {
    id: number;
    accountId: number;
    device: string;
    turned: string;
}
export default function Iot() {
    const [devices, setDevices] = useState<Device[]>([]);

    useEffect(() => {
        const fetchDevices = async () => {
            const accessToken = await getToken();

            try {
                const response = await fetch(API_ROUTES.devices.list, {
                    headers: {
                        "Content-Type": "application/json",
                        Authorization: `Bearer ${accessToken}`
                    }
                });

                if (response.ok) {
                    const data = await response.json();
                    setDevices(data.data || []);
                } else {
                    console.error("기기 조회 실패");
                }
            } catch (error) {
                console.error("기기 조회 오류:", error);
            }
        };
        fetchDevices();
    }, []);

    return (
        <div className="py-3 px-5 w-auto h-44 relative flex items-center justify-center overflow-hidden">
            <div className="relative w-full h-full flex items-center justify-center">
                <div className="relative w-48 h-36">
                    <Image
                        src="/images/iot_map_dark.png"
                        alt="IoT Map"
                        fill
                        style={{ objectFit: "contain" }}
                        className="rounded-md"
                    />

                    {devices.map((device) => {
                        const pos = devicesPos.find((p) => p.device === device.device);
                        if (!pos) return null;
                        const isActive = device.turned === "ON";
                        return (
                            <div
                                key={device.id}
                                style={{ position: "absolute", left: pos.x, top: pos.y }}
                                className={`absolute text-md rounded-full shadow-md whitespace-nowrap
                                ${isActive ? "bg-green-500 text-white" : "bg-gray-400 text-white"}
                            `}
                            >
                                <IoInformationCircleOutline />
                            </div>
                        );
                    })}
                </div>
            </div>
        </div>
    );
}
