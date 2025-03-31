"use client";
import AddSchedule from "@/components/mobile/addSchedule";
import EditSchedule from "@/components/mobile/editSchedule";
import API_ROUTES from "@/config/apiRoutes";
import dayjs from "dayjs";
import isSameOrAfter from "dayjs/plugin/isSameOrAfter";

dayjs.extend(isSameOrAfter);

import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import { AiOutlineEdit } from "react-icons/ai";

interface Schedule {
    id: number;
    title: string;
    profileId: number;
    startDate: string; // ISO 날짜 문자열
    endDate: string;
}

export default function Schedule() {
    const router = useRouter();
    const [schedules, setSchedules] = useState<Schedule[]>([]);
    const [showAddScheduleModal, setShowAddScheduleModal] = useState(false);
    const [showEditScheduleModal, setShowEditScheduleModal] = useState(false);
    const [selectedSchedule, setSelectedSchedule] = useState<Schedule>(schedules[0]);

    const getTokens = async () => {
        let accessToken = localStorage.getItem("accessToken");
        const refreshToken = localStorage.getItem("refreshToken");

        if (!accessToken) {
            router.push("/mobile/login");
            return null;
        }

        try {
            // 1. 액세스 토큰 유효성 확인
            const validateResponse = await fetch(API_ROUTES.auth.validate, {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                    "Authorization": `Bearer ${accessToken}`,
                },
                body: JSON.stringify({ token: accessToken }),
            });

            let validateData = null;
            if (validateResponse.ok) {
                const contentType = validateResponse.headers.get("Content-Type");
                if (contentType && contentType.includes("application/json")) {
                    validateData = await validateResponse.json();
                }
            }

            // 2. 토큰이 만료되었거나 유효하지 않은 경우, 리프레시 토큰으로 새 액세스 토큰 발급
            if (refreshToken) {
                const refreshResponse = await fetch(API_ROUTES.auth.refresh, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json",
                        "Authorization": `Bearer ${accessToken}`,
                    },
                    body: JSON.stringify({ token: refreshToken }),
                });

                let refreshData = null;
                if (refreshResponse.ok) {
                    const contentType = refreshResponse.headers.get("Content-Type");
                    if (contentType && contentType.includes("application/json")) {
                        refreshData = await refreshResponse.json();
                    }
                }

                if (refreshResponse.ok && refreshData.data?.accessToken) {
                    accessToken = refreshData.data.accessToken;
                    localStorage.removeItem("accessToken");
                    if (accessToken) {
                        localStorage.setItem("accessToken", accessToken);
                    }
                    return accessToken;
                } else {
                    localStorage.removeItem("accessToken");
                    localStorage.removeItem("refreshToken");
                    router.push("/mobile/login");
                    return null;
                }
            } else {
                localStorage.removeItem("accessToken");
                router.push("/mobile/login");
                return null;
            }
        } catch (error) {
            console.error("토큰 검증 오류:", error);
            return null;
        }
    };

    const fetchSchedules = async () => {
        const accessToken = await getTokens();
        const profile = JSON.parse(localStorage.getItem("currentProfile") || "{}");
        const profileId = profile?.id;
        if (!accessToken || !profileId) return;

        try {
            const response = await fetch(`${API_ROUTES.schedule.listByProfile}?profileId=${profileId}`, {
                method: "GET",
                headers: {
                    Authorization: `Bearer ${accessToken}`,
                },
            });

            if (response.ok) {
                const data = await response.json();
                setSchedules(data.data || []);
            } else {
                console.error("일정 조회 실패");
            }
        } catch (error) {
            console.error("일정 요청 오류:", error);
        }
    };

    const sortedGroupByDate = (items: Schedule[]) => {
        const today = dayjs().startOf("day"); // 오늘 날짜 00:00

        const filtered = items.filter((item) => dayjs(item.startDate).isSameOrAfter(today));

        const grouped = filtered.reduce((acc: Record<string, Schedule[]>, item) => {
            const dateKey = dayjs(item.startDate).format("MMM D");
            acc[dateKey] = acc[dateKey] || [];
            acc[dateKey].push(item);
            return acc;
        }, {});

        return Object.entries(grouped)
            .sort((a, b) => dayjs(a[1][0].startDate).unix() - dayjs(b[1][0].startDate).unix())
            .reduce((acc: Record<string, Schedule[]>, [key, val]) => {
                acc[key] = val;
                return acc;
            }, {});
    };

    const groupedSchedules = sortedGroupByDate(schedules);

    useEffect(() => {
        fetchSchedules();
        if (typeof window !== "undefined") {
            const token = localStorage.getItem("accessToken"); // 'token' -> 'accessToken'으로 수정
            if (token) {
                const profile = localStorage.getItem("currentProfile"); // 'profile' -> 'currentProfile'으로 수정
                if (!profile) {
                    router.push("/mobile/profile"); // 프로필이 없으면 프로필 페이지로 리다이렉트
                }
            } else {
                router.push("/mobile/login"); // 로그인되어 있지 않으면 로그인 페이지로 리다이렉트
            }
        }
    }, []);

    useEffect(() => {
        fetchSchedules();
    }, [showAddScheduleModal, showEditScheduleModal]);

    return (
        <div className="flex-1 w-full flex h-full items-center justify-center relative">
            <div className="h-full w-full flex flex-col items-center justify-center p-6">
                <div className="flex flex-col h-full space-y-4 p-4 bg-white shadow-md rounded-xl w-full max-w-md overflow-y-auto">
                    <div className="flex justify-between items-center w-full border-b-2 border-blue-300">
                        <h1 className="pl-2 pb-1 text-xl text-blue-300 font-bold">Schedule</h1>
                        <button
                            className="bg-blue-300 rounded-xl text-white px-2 font-bold"
                            onClick={() => setShowAddScheduleModal(true)}
                        >
                            +
                        </button>
                    </div>

                    {/* 날짜별 그룹핑 */}
                    {Object.entries(groupedSchedules).map(([date, items]) => (
                        <div key={date}>
                            <h2 className="text-md text-blue-300 font-semibold pl-2 mb-2">{date}</h2>
                            {items.map((schedule) => (
                                <div
                                    key={schedule.id}
                                    className="flex items-center justify-between px-4 py-2 text-gray-600 bg-blue-100 rounded-full mb-2"
                                >
                                    <span>{schedule.title}</span>
                                    <div className="flex gap-2">
                                        <button
                                            onClick={() => {
                                                console.log(schedule);
                                                setSelectedSchedule(schedule);
                                                setShowEditScheduleModal(true);
                                            }}
                                        >
                                            <AiOutlineEdit />
                                        </button>
                                    </div>
                                </div>
                            ))}
                        </div>
                    ))}
                </div>
            </div>
            {showAddScheduleModal && <AddSchedule onClose={() => setShowAddScheduleModal(false)} />}
            {showEditScheduleModal && (
                <EditSchedule
                    onClose={() => setShowEditScheduleModal(false)}
                    scheduleId={selectedSchedule.id}
                    initialTitle={selectedSchedule.title}
                    initialStartDate={selectedSchedule.startDate} // ISO 형식이어야 함 e.g., '2025-04-01T14:00'
                    initialEndDate={selectedSchedule.endDate}
                />
            )}
        </div>
    );
}
