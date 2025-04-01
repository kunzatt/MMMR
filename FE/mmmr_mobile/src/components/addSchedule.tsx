"use client";
import { useState } from "react";
import { useRouter } from "next/navigation";
import { ImCross } from "react-icons/im";
import API_ROUTES from "@/config/apiRoutes";
import dayjs from "dayjs";

interface AddScheduleProps {
    onClose: () => void;
}

export default function AddSchedule({ onClose }: AddScheduleProps) {
    const router = useRouter();
    const [title, setTitle] = useState("");
    const [startDate, setStartDate] = useState("");
    const [endDate, setEndDate] = useState("");

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
                    localStorage.setItem("accessToken", accessToken);
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

    const handleAddSchedule = async () => {
        const accessToken = await getTokens();
        const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");
        const profileId = profile?.id;

        if (!accessToken || !profileId) {
            alert("로그인이 필요하거나 프로필 정보가 없습니다.");
            router.push("/login");
            return;
        }

        try {
            const response = await fetch(API_ROUTES.schedule.add, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
                body: JSON.stringify({
                    profileId,
                    title,
                    startDate: dayjs(startDate).format("YYYY-MM-DDTHH:mm:ss"),
                    endDate: dayjs(endDate).format("YYYY-MM-DDTHH:mm:ss"),
                }),
            });

            if (response.ok) {
                alert("일정이 추가되었습니다.");
                onClose();
                router.refresh();
            } else {
                const errorData = await response.json();
                console.error("일정 추가 실패:", errorData);
                alert("일정 추가에 실패했습니다.");
            }
        } catch (error) {
            console.error("일정 추가 중 오류:", error);
            alert("네트워크 오류가 발생했습니다.");
        }
    };

    return (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
            <div className="bg-white w-11/12 max-w-sm rounded-lg p-6 relative">
                <h2 className="text-xl text-blue-300 text-center font-bold mb-4">일정 추가</h2>
                <div className="space-y-3">
                    <div>
                        <label className="block text-sm text-gray-500 mb-1">제목</label>
                        <input
                            type="text"
                            value={title}
                            onChange={(e) => setTitle(e.target.value)}
                            className="w-full p-2 border rounded-md"
                            placeholder="새 일정 이름"
                        />
                    </div>
                    <div>
                        <label className="block text-sm text-gray-500 mb-1">시작 날짜</label>
                        <input
                            type="date"
                            value={startDate}
                            onChange={(e) => setStartDate(e.target.value)}
                            className="w-full p-2 border rounded-md"
                        />
                    </div>
                    <div>
                        <label className="block text-sm text-gray-500 mb-1">종료 날짜</label>
                        <input
                            type="date"
                            value={endDate}
                            onChange={(e) => setEndDate(e.target.value)}
                            className="w-full p-2 border rounded-md"
                        />
                    </div>
                </div>
                <div className="flex justify-center mt-6">
                    <button className="bg-blue-300 text-white py-2 px-4 rounded-md w-1/3" onClick={handleAddSchedule}>
                        추가
                    </button>
                    <button className="absolute top-2 right-2 text-gray-500" onClick={onClose}>
                        <ImCross />
                    </button>
                </div>
            </div>
        </div>
    );
}
