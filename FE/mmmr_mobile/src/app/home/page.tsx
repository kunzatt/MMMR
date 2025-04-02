"use client";

import AddDevice from "@/components/addDevice";
import API_ROUTES from "@/config/apiRoutes";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import { HiOutlineTrash } from "react-icons/hi";

interface Device {
    id: number;
    accountId: number;
    device: string;
    turned: string;
}

export default function Home() {
    const router = useRouter();
    const [devices, setDevices] = useState<Device[]>([]);
    const [showAddDeviceModal, setShowAddDeviceModal] = useState(false);

    const getTokens = async () => {
        let accessToken = localStorage.getItem("accessToken");
        const refreshToken = localStorage.getItem("refreshToken");

        if (!accessToken) {
            router.push("/login");
            return null;
        }

        try {
            const validateResponse = await fetch(API_ROUTES.auth.validate, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
                body: JSON.stringify({ token: accessToken }),
            });

            if (!validateResponse.ok && refreshToken) {
                const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`,
                    },
                    body: JSON.stringify({ token: refreshToken }),
                });

                const refreshData = await refreshResponse.json();
                if (refreshResponse.ok && refreshData.data?.accessToken) {
                    accessToken = refreshData.data.accessToken;
                    localStorage.setItem("accessToken", accessToken!);
                    return accessToken;
                } else {
                    localStorage.removeItem("accessToken");
                    localStorage.removeItem("refreshToken");
                    router.push("/login");
                    return null;
                }
            }

            return accessToken;
        } catch (error) {
            console.error("토큰 검증 오류:", error);
            return null;
        }
    };

    const fetchDevices = async () => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(API_ROUTES.devices.list, {
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`,
                },
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

    const deleteDevice = async (deviceId: number) => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(API_ROUTES.devices.delete(deviceId), {
                method: "DELETE",
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`,
                },
            });

            if (response.ok) {
                alert("기기 삭제 완료");
                fetchDevices();
            } else {
                alert("기기 삭제 실패");
            }
        } catch (error) {
            console.error("기기 삭제 오류:", error);
        }
    };

    useEffect(() => {
        if (typeof window !== "undefined") {
            const token = localStorage.getItem("accessToken"); // 'token' -> 'accessToken'으로 수정
            if (token) {
                const profile = localStorage.getItem("currentProfile"); // 'profile' -> 'currentProfile'으로 수정
                if (!profile) {
                    router.push("/profile"); // 프로필이 없으면 프로필 페이지로 리다이렉트
                }
            } else {
                router.push("/login"); // 로그인되어 있지 않으면 로그인 페이지로 리다이렉트
            }
        }
        fetchDevices();
    }, [router]);

    useEffect(() => {
        fetchDevices();
    }, [showAddDeviceModal]);

    return (
        <div className="h-full w-full flex flex-col items-center justify-center p-6">
            <div className="flex flex-col h-full space-y-4 p-4 bg-white shadow-md rounded-xl w-full max-w-md">
                <div className="flex justify-between items-center w-full border-b-2 border-blue-300">
                    <h1 className="pl-2 pb-1 text-xl text-blue-300 font-bold">Iot List</h1>
                    <button
                        className="bg-blue-300 rounded-xl text-white px-2 font-bold"
                        onClick={() => setShowAddDeviceModal(true)}
                    >
                        +
                    </button>
                </div>
                <div className="flex flex-col space-y-2">
                    {devices.map((device) => (
                        <div
                            key={device.id}
                            className={`flex items-center justify-between px-4 py-2 text-gray-600 rounded-full ${
                                device.turned == "ON" ? "bg-blue-100" : "bg-gray-100"
                            }`}
                        >
                            <span className="text-md">{device.device}</span>
                            <div className="flex gap-2">
                                <button className="text-lg" onClick={() => deleteDevice(device.id)}>
                                    <HiOutlineTrash />
                                </button>
                            </div>
                        </div>
                    ))}
                </div>
            </div>
            {showAddDeviceModal && <AddDevice onClose={() => setShowAddDeviceModal(false)} />}
        </div>
    );
}
