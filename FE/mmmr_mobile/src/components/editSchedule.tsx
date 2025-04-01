"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { ImCross } from "react-icons/im";
import API_ROUTES from "@/config/apiRoutes";

interface EditScheduleProps {
    onClose: () => void;
    scheduleId: number;
    initialTitle: string;
    initialStartDate: string;
    initialEndDate: string;
}

export default function EditSchedule({
    onClose,
    scheduleId,
    initialTitle,
    initialStartDate,
    initialEndDate,
}: EditScheduleProps) {
    const router = useRouter();
    const [title, setTitle] = useState(initialTitle);
    const [startDate, setStartDate] = useState(initialStartDate);
    const [endDate, setEndDate] = useState(initialEndDate);

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

            if (validateResponse.ok) {
                return accessToken;
            }

            if (refreshToken) {
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
                }
            }

            localStorage.removeItem("accessToken");
            localStorage.removeItem("refreshToken");
            router.push("/login");
            return null;
        } catch (error) {
            console.error("토큰 오류:", error);
            return null;
        }
    };

    const handleUpdateSchedule = async () => {
        const accessToken = await getTokens();
        const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");

        if (!accessToken || !profile?.id) return;

        try {
            const response = await fetch(API_ROUTES.schedule.update(scheduleId), {
                method: "PUT",
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`,
                },
                body: JSON.stringify({
                    profileId: profile.id,
                    title,
                    startDate: startDate.replace(" ", "T"),
                    endDate: endDate.replace(" ", "T"),
                }),
            });

            if (response.ok) {
                alert("일정이 성공적으로 수정되었습니다.");
                onClose();
                router.refresh();
            } else {
                const errorData = await response.json();
                console.error("에러:", errorData);
                alert("수정 실패");
            }
        } catch (error) {
            console.error("수정 중 오류:", error);
            alert("네트워크 오류");
        }
    };

    const handleDeleteSchedule = async () => {
        const accessToken = await getTokens();
        if (!accessToken) return;

        try {
            const response = await fetch(API_ROUTES.schedule.delete(scheduleId), {
                method: "DELETE",
                headers: {
                    "Content-Type": "application/json",
                    Authorization: `Bearer ${accessToken}`,
                },
            });

            if (response.ok) {
                alert("일정이 삭제되었습니다.");
                onClose();
                router.refresh();
            } else {
                const errorData = await response.json();
                console.error("삭제 에러:", errorData);
                alert("삭제 실패");
            }
        } catch (error) {
            console.error("삭제 중 오류:", error);
            alert("네트워크 오류");
        }
    };

    return (
        <div className="fixed inset-0 bg-black bg-opacity-40 flex items-center justify-center z-50">
            <div className="bg-white w-11/12 max-w-sm rounded-lg p-6 relative">
                <h2 className="text-xl text-blue-300 text-center font-bold mb-4">일정 수정</h2>
                <div className="space-y-3">
                    <div>
                        <label className="block text-sm text-gray-500 mb-1">제목</label>
                        <input
                            type="text"
                            value={title}
                            onChange={(e) => setTitle(e.target.value)}
                            className="w-full border p-2 rounded"
                        />
                    </div>
                    <div>
                        <label className="block text-sm text-gray-500 mb-1">시작 날짜</label>
                        <input
                            type="datetime-local"
                            value={startDate}
                            onChange={(e) => setStartDate(e.target.value)}
                            className="w-full border p-2 rounded"
                        />
                    </div>
                    <div>
                        <label className="block text-sm text-gray-500 mb-1">종료 날짜</label>
                        <input
                            type="datetime-local"
                            value={endDate}
                            onChange={(e) => setEndDate(e.target.value)}
                            className="w-full border p-2 rounded"
                        />
                    </div>
                </div>
                <div className="flex justify-center gap-4 mt-6">
                    <button className="bg-blue-300 text-white px-4 py-2 rounded w-1/3" onClick={handleUpdateSchedule}>
                        수정
                    </button>
                    <button className="bg-red-300 text-white px-4 py-2 rounded w-1/3" onClick={handleDeleteSchedule}>
                        삭제
                    </button>
                    <button className="absolute top-2 right-2 text-gray-500" onClick={onClose}>
                        <ImCross />
                    </button>
                </div>
            </div>
        </div>
    );
}
